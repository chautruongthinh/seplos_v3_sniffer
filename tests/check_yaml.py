"""Static example checks; not a substitute for `esphome config/compile`."""
from pathlib import Path
import ast
import yaml

ROOT = Path(__file__).resolve().parents[1]


class Loader(yaml.SafeLoader):
    pass


Loader.add_constructor("!secret", lambda loader, node: loader.construct_scalar(node))
config = yaml.load((ROOT / "seplos_v3_sniffer.yaml").read_text(), Loader=Loader)
names = [sensor["name"] for sensor in config["sensor"]]
assert len(names) == len(set(names)), "Duplicate sensor names"
for bank in range(2):
    for cell in range(1, 17):
        assert f"${{battery_bank{bank}}} cell_{cell}" in names
for sensor in config["sensor"]:
    assert "filters" not in sensor, "Parser owns the publication interval"
assert config["uart"][0]["rx_pin"] == "GPIO5", "Preserve existing wiring"
assert config["uart"][0]["baud_rate"] == 19200
assert config["uart"][0]["rx_buffer_size"] == 512
for path in (ROOT / "esphome/components/seplos_parser").glob("*.py"):
    ast.parse(path.read_text(), filename=str(path))
schema = (ROOT / "esphome/components/seplos_parser/__init__.py").read_text()
assert "cv.int_range(min=1, max=16)" in schema
assert "cv.int_range(min=0, max=2147483)" in schema
print(f"PASS: YAML structure, {len(names)} unique numeric sensors, Python syntax")
