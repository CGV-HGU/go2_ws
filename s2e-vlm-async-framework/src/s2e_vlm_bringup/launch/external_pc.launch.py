import importlib.util
from pathlib import Path

_SPEC = importlib.util.spec_from_file_location("_launch_helpers", Path(__file__).with_name("_launch_helpers.py"))
if _SPEC is None or _SPEC.loader is None:
    raise ImportError("Unable to load launch helper")
_HELPERS = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_HELPERS)
make_launch_description = _HELPERS.make_launch_description

NODES = ["vlm_node", "e2e_node"]


def generate_launch_description():
    return make_launch_description(NODES, enable_debug_visualizer_default="false")
