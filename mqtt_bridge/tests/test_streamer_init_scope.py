"""Static guard against the `_ensure_ros_streamer` NameError class of bug.

Refactoring the inline WebRTC pre-init into a method once left `resolve_ice_servers`
/`has_turn_server` out of scope (they were imported locally in `__init__`), so every
call raised `NameError` → the camera producer never started. This parses the source
with `ast` (no ROS import needed) and asserts the camera-producer methods reference
no free name that isn't a builtin, a module-level name, or bound within the method.
"""

from __future__ import annotations

import ast
import builtins
from pathlib import Path

NODE_SRC = Path(__file__).resolve().parents[1] / "mqtt_bridge_node.py"

# Methods on the edge WebRTC producer path — a stray free name here means "no stream".
CRITICAL_METHODS = ("_ensure_ros_streamer", "start_camera_stream", "_maybe_auto_start_webrtc")

_BUILTINS = set(dir(builtins))


def _module_level_names(tree: ast.Module) -> set[str]:
    """Names bound at module scope. Recurses through top-level control-flow
    (try/if/with/for) so conditional imports (e.g. `if TYPE_CHECKING:` / `try: import`)
    are captured, but does NOT descend into def/class bodies (those are inner scopes)."""
    names: set[str] = set()

    def visit(body: list) -> None:
        for node in body:
            if isinstance(node, (ast.Import, ast.ImportFrom)):
                for alias in node.names:
                    names.add((alias.asname or alias.name).split(".")[0])
            elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
                names.add(node.name)
            elif isinstance(node, ast.Assign):
                for tgt in node.targets:
                    names.update(n.id for n in ast.walk(tgt) if isinstance(n, ast.Name))
            elif isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
                names.add(node.target.id)
            else:
                for field in ("body", "orelse", "finalbody"):
                    visit(getattr(node, field, []) or [])
                for handler in getattr(node, "handlers", []) or []:
                    visit(handler.body)
    visit(tree.body)
    return names


def _names_bound_in(func: ast.AST) -> set[str]:
    """Everything resolvable inside the function: params, assignments, local imports,
    nested defs, comprehension/except/with targets (over-approx, so no false alarms)."""
    bound: set[str] = set()
    for node in ast.walk(func):
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            for alias in node.names:
                bound.add((alias.asname or alias.name).split(".")[0])
        elif isinstance(node, ast.Name) and isinstance(node.ctx, ast.Store):
            bound.add(node.id)
        elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            bound.add(node.name)
        elif isinstance(node, ast.arg):
            bound.add(node.arg)
        elif isinstance(node, ast.ExceptHandler) and node.name:
            bound.add(node.name)
        elif isinstance(node, ast.Global) or isinstance(node, ast.Nonlocal):
            bound.update(node.names)
    return bound


def _find_func(tree: ast.Module, name: str) -> ast.AST | None:
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name == name:
            return node
    return None


def _loaded_body_names(func: ast.AST) -> set[str]:
    """Names loaded in executable code, EXCLUDING annotation subtrees (annotations
    aren't evaluated at runtime under PEP 563, so they can't cause a NameError)."""
    annotation_node_ids: set[int] = set()
    for node in ast.walk(func):
        anns = []
        if isinstance(node, ast.arg) and node.annotation is not None:
            anns.append(node.annotation)
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.returns:
            anns.append(node.returns)
        if isinstance(node, ast.AnnAssign) and node.annotation is not None:
            anns.append(node.annotation)
        for ann in anns:
            annotation_node_ids.update(id(n) for n in ast.walk(ann))
    return {
        n.id
        for n in ast.walk(func)
        if isinstance(n, ast.Name)
        and isinstance(n.ctx, ast.Load)
        and id(n) not in annotation_node_ids
    }


def _unresolved_names(tree: ast.Module, func: ast.AST) -> set[str]:
    available = _module_level_names(tree) | _names_bound_in(func) | _BUILTINS | {"self"}
    return _loaded_body_names(func) - available


def test_node_source_present():
    assert NODE_SRC.is_file(), f"missing {NODE_SRC}"


def test_camera_producer_methods_have_no_out_of_scope_names():
    tree = ast.parse(NODE_SRC.read_text())
    problems: dict[str, set[str]] = {}
    for name in CRITICAL_METHODS:
        func = _find_func(tree, name)
        assert func is not None, f"method {name}() not found — rename? update this guard"
        unresolved = _unresolved_names(tree, func)
        if unresolved:
            problems[name] = unresolved
    assert not problems, (
        "Camera-producer method(s) reference names not in scope (would NameError at "
        f"runtime → no stream): {problems}. Import/define them in the method or at "
        "module level (this is the resolve_ice_servers regression)."
    )
