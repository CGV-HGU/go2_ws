"""Pin package import locations before legacy test collection adjusts sys.path."""
import hashlib, json, sys
from pathlib import Path
import s2e_vlm_core
import s2e_vlm_nodes.runtime.pixnav as pixnav
import s2e_vlm_robot
import pytest
p=Path(pixnav.__file__)
assert str(p).startswith('/opt/s2e-robot-minimal/'),str(p)
print(json.dumps(dict(installed_pixnav=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest())),flush=True)
raise SystemExit(pytest.main(sys.argv[1:]))
