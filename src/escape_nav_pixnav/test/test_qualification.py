import ast
from pathlib import Path

from escape_nav_pixnav.qualification import audit_runtime_sources


def test_runtime_package_has_no_direct_ros_socket_or_unitree_dependency():
    package_dir = Path(__file__).resolve().parents[1] / "escape_nav_pixnav"
    result = audit_runtime_sources(package_dir)

    assert result["passed"] is True
    assert result["missing_modules"] == []
    assert result["forbidden_import_roots"] == []
    assert len(result["source_sha256"]) == len(result["scope"])


def test_static_audit_detects_socket_import(tmp_path):
    package_dir = tmp_path / "runtime"
    package_dir.mkdir()
    safe_source = "VALUE = 1\n"
    from escape_nav_pixnav.qualification import RUNTIME_MODULES

    for name in RUNTIME_MODULES:
        (package_dir / name).write_text(safe_source, encoding="utf-8")
    (package_dir / RUNTIME_MODULES[0]).write_text("import socket\n", encoding="utf-8")

    result = audit_runtime_sources(package_dir)

    assert result["passed"] is False
    assert result["forbidden_import_roots"] == ["socket"]


def test_qualification_module_itself_is_syntax_valid():
    path = Path(__file__).resolve().parents[1] / "escape_nav_pixnav" / "qualification.py"
    ast.parse(path.read_text(encoding="utf-8"))
