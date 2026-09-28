#!/usr/bin/env bash
#
# expand_rootfs.sh — grow the root partition + filesystem to fill the disk.
#
# Pi OS should auto-expand root on first boot; when it doesn't, the driver runs
# out of disk and builds fail (ENOSPC). This reclaims the unallocated space.
# SAFE + idempotent: finds / and its parent disk (no hardcoded device); if there
# is free space after root, grows it (growpart) + resizes the fs (resize2fs,
# online); otherwise does nothing. Only ever grows the LAST partition; never
# deletes one. Boot/data partitions untouched.
#
# Usage: sudo ./expand_rootfs.sh [--dry-run|--install|--uninstall]
# --install enables a systemd one-shot that runs this once at boot (no-op when full).

set -euo pipefail

LOG_FILE="/var/log/ugv_expand_rootfs.log"
SERVICE_NAME="ugv-expand-rootfs.service"
INSTALL_PATH="/usr/local/sbin/ugv-expand-rootfs"
DRY_RUN=0

log() {
    local msg="[$(date '+%Y-%m-%dT%H:%M:%S%z')] $*"
    echo "$msg"
    # Best-effort file log (may not be writable in some early-boot contexts).
    echo "$msg" >>"$LOG_FILE" 2>/dev/null || true
}

die() {
    log "ERROR: $*"
    exit 1
}

require_root() {
    if [ "$(id -u)" -ne 0 ]; then
        die "must run as root (try: sudo $0 ${*:-})"
    fi
}

# --------------------------------------------------------------------------- #
# Discover the root partition, its parent disk, and the partition number.
# Handles both sdX2 and mmcblk0p2 / nvme0n1p2 naming.
# Sets globals: ROOT_SRC, DISK, PARTNUM, FSTYPE
# --------------------------------------------------------------------------- #
discover_root() {
    ROOT_SRC="$(findmnt -no SOURCE / || true)"
    [ -n "${ROOT_SRC}" ] || die "could not determine the source device of /"
    ROOT_SRC="$(readlink -f "${ROOT_SRC}")"

    # Parent kernel device name (e.g. mmcblk0, sda, nvme0n1).
    local pkname
    pkname="$(lsblk -no PKNAME "${ROOT_SRC}" 2>/dev/null | sed -n '1p' || true)"
    [ -n "${pkname}" ] || die "could not resolve parent disk of ${ROOT_SRC} (is / on an LVM/overlay?)"
    DISK="/dev/${pkname}"
    [ -b "${DISK}" ] || die "parent disk ${DISK} is not a block device"

    # Partition number = trailing digits after the disk name (strip optional 'p').
    local suffix="${ROOT_SRC#/dev/${pkname}}"   # e.g. "p2" or "2"
    PARTNUM="$(printf '%s' "${suffix}" | grep -oE '[0-9]+$' || true)"
    [ -n "${PARTNUM}" ] || die "could not determine partition number from ${ROOT_SRC}"

    FSTYPE="$(findmnt -no FSTYPE / || true)"
}

# --------------------------------------------------------------------------- #
# Decide whether there is free space to grow into. Returns 0 if expansion is
# possible, 1 if nothing to do. Uses growpart's dry-run (authoritative).
# --------------------------------------------------------------------------- #
expansion_possible() {
    local out
    set +e
    out="$(growpart --dry-run "${DISK}" "${PARTNUM}" 2>&1)"
    set -e
    log "growpart dry-run: ${out}"
    case "${out}" in
        *NOCHANGE*) return 1 ;;
        *CHANGE*)   return 0 ;;
        *)
            # Unknown output — be conservative and treat as nothing to do.
            log "unrecognized growpart output; treating as NOCHANGE"
            return 1
            ;;
    esac
}

ensure_growpart() {
    if command -v growpart >/dev/null 2>&1; then
        return 0
    fi
    log "growpart not found; attempting to install cloud-guest-utils (best effort)"
    if command -v apt-get >/dev/null 2>&1; then
        DEBIAN_FRONTEND=noninteractive apt-get update -y >>"$LOG_FILE" 2>&1 || true
        DEBIAN_FRONTEND=noninteractive apt-get install -y cloud-guest-utils >>"$LOG_FILE" 2>&1 || true
    fi
    command -v growpart >/dev/null 2>&1 || \
        die "growpart unavailable (install 'cloud-guest-utils'); cannot expand safely"
}

grow_filesystem() {
    case "${FSTYPE}" in
        ext2|ext3|ext4)
            log "resizing ${FSTYPE} filesystem on ${ROOT_SRC} (online)"
            resize2fs "${ROOT_SRC}" >>"$LOG_FILE" 2>&1
            ;;
        btrfs)
            log "resizing btrfs filesystem mounted at /"
            btrfs filesystem resize max / >>"$LOG_FILE" 2>&1
            ;;
        f2fs)
            # f2fs typically needs the fs unmounted; do not risk it online at boot.
            log "f2fs detected; partition grown but filesystem NOT resized online. "
            log "Run 'resize.f2fs ${ROOT_SRC}' from a context where / is unmounted."
            ;;
        *)
            log "unknown root fstype '${FSTYPE}'; partition grown but fs not resized"
            ;;
    esac
}

do_expand() {
    require_root
    discover_root
    log "root=${ROOT_SRC} disk=${DISK} partnum=${PARTNUM} fstype=${FSTYPE}"

    ensure_growpart

    if ! expansion_possible; then
        log "no unallocated space after the root partition — nothing to do."
        exit 0
    fi

    if [ "${DRY_RUN}" -eq 1 ]; then
        log "DRY-RUN: would grow ${DISK} partition ${PARTNUM} and resize ${FSTYPE}; no changes made."
        exit 0
    fi

    log "growing partition ${PARTNUM} on ${DISK} ..."
    # growpart updates the kernel partition table (online) for the mounted root.
    growpart "${DISK}" "${PARTNUM}" >>"$LOG_FILE" 2>&1 || \
        die "growpart failed (see ${LOG_FILE})"

    grow_filesystem

    log "done. Current usage:"
    df -h / | tee -a "$LOG_FILE"
}

# --------------------------------------------------------------------------- #
# systemd boot integration
# --------------------------------------------------------------------------- #
install_service() {
    require_root --install
    log "installing ${INSTALL_PATH} and ${SERVICE_NAME}"
    install -m 0755 "$0" "${INSTALL_PATH}"

    cat >"/etc/systemd/system/${SERVICE_NAME}" <<UNIT
[Unit]
Description=Expand UGV root filesystem to fill the disk (one-shot, idempotent)
DefaultDependencies=no
After=local-fs.target
# Make sure space is reclaimed before Docker / the driver start.
Before=docker.service network-pre.target
ConditionVirtualization=!container

[Service]
Type=oneshot
ExecStart=${INSTALL_PATH}
RemainAfterExit=yes
StandardOutput=journal+console

[Install]
WantedBy=multi-user.target
UNIT

    systemctl daemon-reload
    systemctl enable "${SERVICE_NAME}"
    log "enabled ${SERVICE_NAME}. It will run on the next boot (and is a no-op once full)."
    log "To expand right now without rebooting: sudo systemctl start ${SERVICE_NAME}"
}

uninstall_service() {
    require_root --uninstall
    systemctl disable "${SERVICE_NAME}" 2>/dev/null || true
    rm -f "/etc/systemd/system/${SERVICE_NAME}"
    rm -f "${INSTALL_PATH}"
    systemctl daemon-reload 2>/dev/null || true
    log "uninstalled ${SERVICE_NAME}"
}

usage() {
    sed -n '3,13p' "$0"
}

main() {
    case "${1:-}" in
        ""|--run)      do_expand ;;
        --dry-run)     DRY_RUN=1; do_expand ;;
        --install)     install_service ;;
        --uninstall)   uninstall_service ;;
        -h|--help)     usage ;;
        *)             die "unknown argument: $1 (use --help)";;
    esac
}

main "$@"
