"""Static example checks; not a substitute for `esphome config/compile`."""
from pathlib import Path
import ast
import hashlib
import yaml

ROOT = Path(__file__).resolve().parents[1]


class Loader(yaml.SafeLoader):
    pass


Loader.add_constructor("!secret", lambda loader, node: loader.construct_scalar(node))
config = yaml.load((ROOT / "seplos_v3_sniffer.yaml").read_text(), Loader=Loader)
names = [sensor["name"] for sensor in config["sensor"]]
for bank in range(2):
    for cell in range(1, 17):
        assert f"${{battery_bank{bank}}} cell_{cell}" in names
# Preserve the user's original example exactly, including its existing filters.
raw = (ROOT / "seplos_v3_sniffer.yaml").read_bytes()
blob = b"blob " + str(len(raw)).encode() + b"\0" + raw
assert hashlib.sha1(blob).hexdigest() == "9160ea5d816cdb5731ae77f0e406e5a0bf657268"
for path in (ROOT / "esphome/components/seplos_parser").glob("*.py"):
    ast.parse(path.read_text(), filename=str(path))
schema = (ROOT / "esphome/components/seplos_parser/__init__.py").read_text()
assert "cv.int_range(min=1, max=16)" in schema
assert "cv.int_range(min=0, max=2147483)" in schema
print(f"PASS: YAML structure, unchanged original configuration, Python syntax")
