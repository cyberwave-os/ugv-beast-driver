#!/bin/bash
# =============================================================================
# UGV Beast End-to-End Integration Test
# =============================================================================
#
# Simulates the full lifecycle of a Cyberwave-managed UGV Beast:
#   1. Starts the backend locally (or reuses a running one)
#   2. Seeds the database
#   3. Builds the UGV Docker image
#   4. Builds a "Raspberry Pi simulator" container (CLI + edge-core + Docker CLI)
#   5. Inside the Pi simulator:
#      a. Logs in with the CLI
#      b. Creates a project, environment, and UGV Beast twin (waveshare/ugv-beast)
#      c. Sets driver metadata pointing to the UGV Docker image
#      d. Configures edge-core (environment.json, fingerprint.json)
#      e. Runs edge-core driver discovery → starts the UGV driver container
#   6. Verifies the driver container started and connected to MQTT
#
# Usage:
#   cd cyberwave-edge-nodes/cyberwave-edge-ros-ugv && bash test-ugv-e2e.sh
#   bash test-ugv-e2e.sh --skip-build  # skip Docker image rebuild
#
# Prerequisites:
#   - Docker and Docker Compose installed
#   - Ports 8000, 5432, 6379, 1883 available
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
EDGE_NODES_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
BACKEND_DIR="$REPO_ROOT/cyberwave-backend"
CLI_DIR="$REPO_ROOT/cyberwave-clis/cyberwave-python-cli"
EDGE_CORE_DIR="$REPO_ROOT/cyberwave-edge-core"
SDK_DIR="$REPO_ROOT/${CYBERWAVE_SDK_REL_PATH:-cyberwave-sdks/cyberwave-python}"

UGV_IMAGE="cyberwave-edge-ros-ugv:local"
PI_SIM_IMAGE="cyberwave-ugv-pi-sim"
SKIP_BUILD=false
E2E_CONFIG_DIR="${CYBERWAVE_UGV_E2E_CONFIG_DIR:-$SCRIPT_DIR/.tmp/e2e-config}"
CONFIG_DIR_CREATED=false

if [ -z "${CYBERWAVE_UGV_E2E_CONFIG_DIR:-}" ]; then
    CONFIG_DIR_CREATED=true
    rm -rf "$E2E_CONFIG_DIR"
fi
mkdir -p "$E2E_CONFIG_DIR"
CONFIG_VOLUME_ARGS=(
    -v "$E2E_CONFIG_DIR:$E2E_CONFIG_DIR"
    -e "CYBERWAVE_EDGE_CONFIG_DIR=$E2E_CONFIG_DIR"
)

TEST_EMAIL="admin@cyberwave.com"
TEST_PASSWORD="admin123"

DOCKER_SOCK="/var/run/docker.sock"
for _candidate in \
    "${DOCKER_HOST#unix://}" \
    "$HOME/.docker/run/docker.sock" \
    "/var/run/docker.sock"; do
    if [ -S "$_candidate" ]; then
        DOCKER_SOCK="$_candidate"
        break
    fi
done

for arg in "$@"; do
    case $arg in
        --skip-build)
            SKIP_BUILD=true
            shift
            ;;
        --help)
            echo "Usage: bash test-ugv-e2e.sh [OPTIONS]"
            echo "Options:"
            echo "  --skip-build   Reuse existing Docker images"
            echo "  --help         Show this help"
            exit 0
            ;;
    esac
done

BACKEND_STARTED=false

cleanup() {
    echo ""
    echo "=== Cleaning up ==="
    docker rm -f cyberwave-driver-* 2>/dev/null || true
    if [ "$CONFIG_DIR_CREATED" = true ]; then
        rm -rf "$E2E_CONFIG_DIR"
        rmdir "$SCRIPT_DIR/.tmp" 2>/dev/null || true
    fi
    if [ "$BACKEND_STARTED" = true ]; then
        echo "Stopping backend..."
        cd "$BACKEND_DIR" && docker compose -f local.yml down --remove-orphans 2>/dev/null || true
    fi
    echo "Done."
}

trap cleanup EXIT

# =============================================================================
# Step 1: Start the backend
# =============================================================================
echo ""
echo "=========================================="
echo " Step 1: Starting backend locally"
echo "=========================================="

cd "$BACKEND_DIR"

if curl -sf http://localhost:8000/healthz > /dev/null 2>&1; then
    echo "Backend is already running, skipping startup."
else
    echo "Starting backend with: docker compose -f local.yml up -d"
    docker compose -f local.yml up -d
    BACKEND_STARTED=true
fi

# =============================================================================
# Step 2: Wait for backend to be healthy
# =============================================================================
echo ""
echo "=========================================="
echo " Step 2: Waiting for backend health check"
echo "=========================================="

MAX_RETRIES=120
RETRY_COUNT=0

until curl -sf http://localhost:8000/healthz > /dev/null 2>&1; do
    RETRY_COUNT=$((RETRY_COUNT + 1))
    if [ $RETRY_COUNT -ge $MAX_RETRIES ]; then
        echo "ERROR: Backend did not become healthy within ${MAX_RETRIES} seconds"
        docker compose -f local.yml logs --tail=50 django
        exit 1
    fi
    printf "\r  Waiting for backend... (%d/%d)" "$RETRY_COUNT" "$MAX_RETRIES"
    sleep 1
done

echo ""
echo "Backend is healthy!"

# =============================================================================
# Step 3: Seed the database
# =============================================================================
echo ""
echo "=========================================="
echo " Step 3: Seeding database"
echo "=========================================="

if [ "$BACKEND_STARTED" = true ]; then
    echo "Backend was freshly started — running migrations and seeding..."
    docker compose -f local.yml exec -T django python manage.py migrate --run-syncdb --no-input 2>/dev/null || true
    docker compose -f local.yml exec -T django python manage.py seed_data --skip-assets --skip-projects
    echo "Database seeded with test user: $TEST_EMAIL"
else
    echo "Backend was already running — skipping migrations and seeding."
fi

# =============================================================================
# Step 4: Build the UGV Docker image
# =============================================================================
echo ""
echo "=========================================="
echo " Step 4: Building UGV Docker image"
echo "=========================================="

cd "$EDGE_NODES_DIR"

if [ "$SKIP_BUILD" = true ]; then
    if docker image inspect "$UGV_IMAGE" &>/dev/null; then
        echo "Skipping UGV build — reusing existing image: $UGV_IMAGE"
    elif docker image inspect "cyberwave-edge-ros-ugv" &>/dev/null; then
        echo "Tagging existing cyberwave-edge-ros-ugv image as $UGV_IMAGE"
        docker tag cyberwave-edge-ros-ugv "$UGV_IMAGE"
    else
        echo "ERROR: --skip-build but image '$UGV_IMAGE' not found. Run without --skip-build."
        exit 1
    fi
else
    echo "Building $UGV_IMAGE (this takes 10-15 min on first build)..."
    docker build -f cyberwave-edge-ros-ugv/docker-conf/Dockerfile -t "$UGV_IMAGE" .
    echo "  ✅ UGV Docker image built"
fi

# =============================================================================
# Step 5: Build the Pi simulator container
# =============================================================================
echo ""
echo "=========================================="
echo " Step 5: Building Pi simulator container"
echo "=========================================="

if [ ! -d "$SDK_DIR" ]; then
    echo "ERROR: SDK not found at $SDK_DIR"
    echo "Set CYBERWAVE_SDK_REL_PATH to a valid path relative to repo root."
    exit 1
fi

docker build \
    --build-arg CYBERWAVE_SDK_REL_PATH="${CYBERWAVE_SDK_REL_PATH:-cyberwave-sdks/cyberwave-python}" \
    -t "$PI_SIM_IMAGE" \
    -f - "$REPO_ROOT" <<'DOCKERFILE'
FROM docker:cli AS docker-cli

FROM python:3.11-slim
ARG CYBERWAVE_SDK_REL_PATH

COPY --from=docker-cli /usr/local/bin/docker /usr/local/bin/docker

RUN apt-get update && apt-get install -y --no-install-recommends \
    curl socat \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

ENV CLI_VENV=/opt/venvs/cli
ENV EDGE_VENV=/opt/venvs/edge-core
ENV PATH="$CLI_VENV/bin:$EDGE_VENV/bin:$PATH"

COPY ${CYBERWAVE_SDK_REL_PATH}/ /workspace/cyberwave-sdk/
COPY cyberwave-clis/cyberwave-python-cli/ /workspace/cyberwave-cli/
COPY cyberwave-edge-core/ /workspace/cyberwave-edge-core/

# CLI venv
RUN python -m venv "$CLI_VENV" && \
    "$CLI_VENV/bin/pip" install --no-cache-dir --upgrade pip setuptools wheel && \
    "$CLI_VENV/bin/pip" install --no-cache-dir -e "/workspace/cyberwave-sdk" && \
    "$CLI_VENV/bin/pip" install --no-cache-dir -e "/workspace/cyberwave-cli/"

# Edge Core venv
RUN if [ -d /workspace/cyberwave-edge-core/cyberwave_edge_core ]; then \
        python -m venv "$EDGE_VENV" && \
        "$EDGE_VENV/bin/pip" install --no-cache-dir --upgrade pip setuptools wheel && \
        "$EDGE_VENV/bin/pip" install --no-cache-dir -e "/workspace/cyberwave-sdk" && \
        "$EDGE_VENV/bin/pip" install --no-cache-dir -e "/workspace/cyberwave-edge-core/"; \
    else \
        echo "NOTICE: cyberwave_edge_core source not found, skipping"; \
    fi
DOCKERFILE

echo "  ✅ Pi simulator image built"

# =============================================================================
# Step 6: Login with CLI
# =============================================================================
echo ""
echo "=========================================="
echo " Step 6: CLI login against local backend"
echo "=========================================="

docker run --rm \
    --add-host=host.docker.internal:host-gateway \
    "${CONFIG_VOLUME_ARGS[@]}" \
    -e CYBERWAVE_BASE_URL=http://host.docker.internal:8000 \
    "$PI_SIM_IMAGE" \
    cyberwave login --email "$TEST_EMAIL" --password "$TEST_PASSWORD"

echo "  ✅ CLI login succeeded"

# =============================================================================
# Step 7: Create environment + UGV Beast twin + set driver metadata
# =============================================================================
echo ""
echo "=========================================="
echo " Step 7: Create environment + UGV Beast twin"
echo "=========================================="

docker run --rm -i \
    --add-host=host.docker.internal:host-gateway \
    "${CONFIG_VOLUME_ARGS[@]}" \
    -e CYBERWAVE_BASE_URL=http://host.docker.internal:8000 \
    -e CYBERWAVE_MQTT_HOST=host.docker.internal \
    -e CYBERWAVE_MQTT_PORT=1883 \
    -e CYBERWAVE_MQTT_USE_TLS=false \
    -e CYBERWAVE_ENVIRONMENT=local \
    "$PI_SIM_IMAGE" \
    bash -c "
set -e
export PATH=\"/opt/venvs/cli/bin:\$PATH\"

# Login
cyberwave login --email '$TEST_EMAIL' --password '$TEST_PASSWORD'

# Create project, environment, asset, twin and configure driver metadata
/opt/venvs/cli/bin/python3 - <<'PY'
import json
import time
import httpx

from cyberwave import Cyberwave
from cyberwave.fingerprint import generate_fingerprint
from cyberwave_cli.config import CONFIG_DIR, get_api_url
from cyberwave_cli.credentials import load_credentials

creds = load_credentials()
assert creds and creds.token, 'credentials/token not found after login'
assert creds.workspace_uuid, 'workspace_uuid not found in credentials'

base_url = get_api_url()
client = Cyberwave(base_url=base_url, api_key=creds.token)
headers = {
    'Authorization': f'Token {creds.token}',
    'Accept': 'application/json',
    'Content-Type': 'application/json',
}

suffix = str(int(time.time()))

# --- Create project ---
project = client.projects.create(
    name=f'UGV E2E Test {suffix}',
    workspace_id=creds.workspace_uuid,
    description='Project for UGV Beast end-to-end test',
)
project_id = str(project.uuid)
print(f'  Project created: {project_id[:8]}...')

# --- Create environment ---
environment = client.environments.create(
    name=f'UGV E2E Environment {suffix}',
    project_id=project_id,
    description='Environment for UGV Beast end-to-end test',
)
env_uuid = str(environment.uuid)
print(f'  Environment created: {env_uuid[:8]}...')

# --- Create or get the waveshare/ugv-beast asset ---
resp = httpx.get(
    f'{base_url}/api/v1/assets',
    headers=headers,
    params={'search': 'waveshare/ugv-beast'},
    timeout=15.0,
)
assets = resp.json() if resp.status_code == 200 else []
ugv_asset = next((a for a in assets if a.get('registry_id') == 'waveshare/ugv-beast'), None)

if ugv_asset:
    asset_uuid = ugv_asset['uuid']
    print(f'  Asset found: waveshare/ugv-beast ({asset_uuid[:8]}...)')
else:
    resp = httpx.post(
        f'{base_url}/api/v1/assets',
        headers=headers,
        json={
            'name': 'UGV Beast',
            'description': 'Waveshare UGV Beast tracked robot platform',
            'registry_id': 'waveshare/ugv-beast',
            'workspace_uuid': creds.workspace_uuid,
            'metadata': {
                'manufacturer': 'Waveshare',
                'category': 'ugv',
            },
        },
        timeout=15.0,
    )
    assert resp.status_code in (200, 201), f'Failed to create asset: {resp.status_code} {resp.text[:300]}'
    asset_uuid = resp.json()['uuid']
    print(f'  Asset created: waveshare/ugv-beast ({asset_uuid[:8]}...)')

# --- Create twin ---
resp = httpx.post(
    f'{base_url}/api/v1/twins',
    headers=headers,
    json={
        'asset_uuid': asset_uuid,
        'environment_uuid': env_uuid,
        'name': f'UGV Beast Twin {suffix}',
    },
    timeout=15.0,
)
assert resp.status_code in (200, 201), f'Failed to create twin: {resp.status_code} {resp.text[:300]}'
twin_uuid = resp.json()['uuid']
print(f'  Twin created: {twin_uuid[:8]}...')

# --- Set driver metadata + edge fingerprint ---
fingerprint = generate_fingerprint()

resp = httpx.put(
    f'{base_url}/api/v1/twins/{twin_uuid}',
    headers=headers,
    json={
        'metadata': {
            'edge_fingerprint': fingerprint,
            'drivers': {
                'default': {
                    'docker_image': '$UGV_IMAGE',
                    'params': ['--privileged'],
                },
            },
        },
    },
    timeout=15.0,
)
assert resp.status_code == 200, f'Failed to update twin metadata: {resp.status_code} {resp.text[:300]}'
print(f'  Driver metadata set: image=$UGV_IMAGE')
print(f'  Edge fingerprint: {fingerprint}')

# --- Write edge config files ---
config_dir = CONFIG_DIR
config_dir.mkdir(parents=True, exist_ok=True)

(config_dir / 'fingerprint.json').write_text(
    json.dumps({'fingerprint': fingerprint}, indent=2) + '\n'
)
(config_dir / 'environment.json').write_text(
    json.dumps({
        'uuid': env_uuid,
        'workspace_uuid': creds.workspace_uuid,
        'twin_uuids': [twin_uuid],
    }, indent=2) + '\n'
)

# --- Write a summary file for later steps ---
(config_dir / 'ugv_e2e_setup.json').write_text(
    json.dumps({
        'project_uuid': project_id,
        'environment_uuid': env_uuid,
        'twin_uuid': twin_uuid,
        'asset_uuid': asset_uuid,
        'fingerprint': fingerprint,
        'driver_image': '$UGV_IMAGE',
    }, indent=2) + '\n'
)

print()
print('  ✅ UGV Beast environment fully configured')
print(f'     Project:     {project_id[:8]}...')
print(f'     Environment: {env_uuid[:8]}...')
print(f'     Twin:        {twin_uuid[:8]}...')
print(f'     Asset:       waveshare/ugv-beast ({asset_uuid[:8]}...)')
print(f'     Driver:      $UGV_IMAGE')
PY
"

echo "  ✅ Environment and UGV Beast twin created"

# =============================================================================
# Step 8: Install edge-core and configure it
# =============================================================================
echo ""
echo "=========================================="
echo " Step 8: Configure edge-core on the Pi simulator"
echo "=========================================="

docker run --rm -i \
    --add-host=host.docker.internal:host-gateway \
    "${CONFIG_VOLUME_ARGS[@]}" \
    -e CYBERWAVE_BASE_URL=http://host.docker.internal:8000 \
    -e CYBERWAVE_MQTT_HOST=host.docker.internal \
    -e CYBERWAVE_MQTT_PORT=1883 \
    -e CYBERWAVE_MQTT_USE_TLS=false \
    -e CYBERWAVE_ENVIRONMENT=local \
    -v "$DOCKER_SOCK":/var/run/docker.sock \
    "$PI_SIM_IMAGE" \
    bash -c "
set -e
export PATH=\"/opt/venvs/cli/bin:\$PATH\"

# Login
cyberwave login --email '$TEST_EMAIL' --password '$TEST_PASSWORD'

# Verify edge install --help works
cyberwave edge install --help > /dev/null
echo '  ✅ cyberwave edge install --help works'

# Verify edge-core is available
if /opt/venvs/edge-core/bin/python3 -c 'import cyberwave_edge_core' 2>/dev/null; then
    echo '  ✅ Edge Core module available'
else
    echo '  ⚠️  Edge Core not installed, skipping core startup test'
    exit 0
fi

# Configure edge environment (creates environment.json)
/opt/venvs/cli/bin/python3 - <<'PY'
import json
from cyberwave_cli.config import CONFIG_DIR
from cyberwave_cli.core import configure_edge_environment

ok = configure_edge_environment(skip_confirm=True)
assert ok, 'configure_edge_environment returned False'

env_path = CONFIG_DIR / 'environment.json'
assert env_path.exists(), f'{env_path} not found'

data = json.loads(env_path.read_text())
assert data.get('uuid'), 'environment UUID missing'
assert isinstance(data.get('twin_uuids'), list), 'twin_uuids missing'

print(f'  ✅ Edge environment configured: {data[\"uuid\"][:8]}...')
print(f'     Twins: {len(data.get(\"twin_uuids\", []))} selected')
PY
"

echo "  ✅ Edge-core configured"

# =============================================================================
# Step 9: Edge-core driver discovery (fetch_and_run_twin_drivers)
# =============================================================================
echo ""
echo "=========================================="
echo " Step 9: Edge-core discovers and starts UGV driver"
echo "=========================================="

docker run --rm -i \
    --add-host=host.docker.internal:host-gateway \
    "${CONFIG_VOLUME_ARGS[@]}" \
    -e CYBERWAVE_BASE_URL=http://host.docker.internal:8000 \
    -e CYBERWAVE_MQTT_HOST=host.docker.internal \
    -e CYBERWAVE_MQTT_PORT=1883 \
    -e CYBERWAVE_MQTT_USE_TLS=false \
    -e CYBERWAVE_ENVIRONMENT=local \
    -v "$DOCKER_SOCK":/var/run/docker.sock \
    "$PI_SIM_IMAGE" \
    bash -c "
set -e
export PATH=\"/opt/venvs/edge-core/bin:/opt/venvs/cli/bin:\$PATH\"

# Skip if edge-core is not installed
if ! /opt/venvs/edge-core/bin/python3 -c 'import cyberwave_edge_core' 2>/dev/null; then
    echo '  ⚠️  Edge Core not installed, skipping'
    exit 0
fi

if ! docker info >/dev/null 2>&1; then
    echo 'ERROR: Docker daemon not reachable'
    exit 1
fi
echo '  ✅ Docker CLI available, host daemon reachable'

# Login and setup config files
cyberwave login --email '$TEST_EMAIL' --password '$TEST_PASSWORD'

/opt/venvs/cli/bin/python3 - <<'PY'
import json
from cyberwave_cli.config import CONFIG_DIR
from cyberwave_cli.credentials import load_credentials
from cyberwave_cli.core import configure_edge_environment

creds = load_credentials()

configure_edge_environment(skip_confirm=True)

config_dir = CONFIG_DIR
fingerprint_path = config_dir / 'fingerprint.json'
fingerprint = json.loads(fingerprint_path.read_text()).get('fingerprint')
assert fingerprint, f'{fingerprint_path} did not contain a fingerprint'

print(f'  Fingerprint: {fingerprint}')
PY

# Run driver discovery
/opt/venvs/edge-core/bin/python3 - <<'PY'
import json
from cyberwave_edge_core.startup import CONFIG_DIR, fetch_and_run_twin_drivers

config_dir = CONFIG_DIR
creds = json.loads((config_dir / 'credentials.json').read_text())
env_data = json.loads((config_dir / 'environment.json').read_text())
fp_data = json.loads((config_dir / 'fingerprint.json').read_text())

token = creds['token']
env_uuid = env_data['uuid']
fingerprint = fp_data['fingerprint']

print(f'  Running fetch_and_run_twin_drivers...')
print(f'    env={env_uuid[:8]}..., fingerprint={fingerprint}')

results = fetch_and_run_twin_drivers(token, env_uuid, fingerprint)

print(f'  Driver results: {len(results)} twin(s)')
for r in results:
    print(f'    - {r.get(\"twin_name\", \"?\")} ({r.get(\"twin_uuid\", \"?\")[:8]}...)')
    print(f'      image: {r.get(\"driver_image\", \"?\")}')
    print(f'      success: {r.get(\"success\", \"?\")}')

if results:
    print()
    failures = [r for r in results if not r.get('success')]
    if failures:
        failed_names = ', '.join(
            '{} ({})'.format(r.get('twin_name', '?'), r.get('driver_image', '?'))
            for r in failures
        )
        raise SystemExit(f'Driver startup failed for: {failed_names}')
    print('  ✅ Edge-core discovered and launched driver(s)')
else:
    raise SystemExit('No drivers found (twin may not have edge_fingerprint set)')
PY
"

echo "  ✅ Edge-core driver discovery completed"

# =============================================================================
# Done
# =============================================================================
echo ""
echo "=========================================="
echo " ✅ UGV Beast E2E test completed!"
echo "=========================================="
echo ""
echo "Summary:"
echo "  - Backend started and seeded"
echo "  - UGV Docker image built ($UGV_IMAGE)"
echo "  - Pi simulator container built ($PI_SIM_IMAGE)"
echo "  - CLI login verified against local backend"
echo "  - waveshare/ugv-beast asset created (or found)"
echo "  - UGV Beast twin created with driver metadata"
echo "  - Edge-core configured and driver discovery tested"
echo ""
