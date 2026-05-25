"""Loader for `pi_slots.json`: maps `/robot/mode_cmd` Int8 codes 80..89 to
scripts in `scripts/<name>.script`.

JSON instead of TOML because the repo's Python is 3.10 in places (no stdlib
`tomllib`) and we don't want a new dependency for ten lines of config.

Schema (all fields optional except slots[i].index and slots[i].script):
    {
      "defaults": { "pen_s": 0.0 },
      "slots": [
        {"index": 0, "script": "square_trace", "label": "Square (trace)"},
        {"index": 1, "script": "circle"}
      ]
    }

Validation: index in [0, 9], unique across slots, script must resolve via
`script.load_script` (file must exist on disk). pen_s defaults to
`defaults.pen_s` (in turn defaulting to 0.0).
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

from .script import SCRIPTS_DIR


SLOT_MIN = 0
SLOT_MAX = 9


@dataclass(frozen=True)
class SlotEntry:
    index: int
    script: str
    label: str
    pen_s: float


class SlotsConfigError(ValueError):
    pass


def load_slots(path: Path | str) -> dict[int, SlotEntry]:
    p = Path(path)
    try:
        raw = json.loads(p.read_text())
    except FileNotFoundError as e:
        raise SlotsConfigError(f"slots config not found: {p}") from e
    except json.JSONDecodeError as e:
        raise SlotsConfigError(f"{p}: invalid JSON ({e})") from e
    return parse_slots(raw, scripts_dir=SCRIPTS_DIR)


def parse_slots(raw: dict, scripts_dir: Path) -> dict[int, SlotEntry]:
    defaults = raw.get("defaults") or {}
    default_pen_s = float(defaults.get("pen_s", 0.0))

    slots_list = raw.get("slots")
    if not isinstance(slots_list, list):
        raise SlotsConfigError("missing or non-list 'slots' field")

    out: dict[int, SlotEntry] = {}
    for i, entry in enumerate(slots_list):
        if not isinstance(entry, dict):
            raise SlotsConfigError(f"slots[{i}]: must be an object")
        if "index" not in entry:
            raise SlotsConfigError(f"slots[{i}]: missing 'index'")
        if "script" not in entry:
            raise SlotsConfigError(f"slots[{i}]: missing 'script'")
        idx = entry["index"]
        if not isinstance(idx, int) or idx < SLOT_MIN or idx > SLOT_MAX:
            raise SlotsConfigError(
                f"slots[{i}]: index must be int in [{SLOT_MIN}, {SLOT_MAX}], got {idx!r}"
            )
        if idx in out:
            raise SlotsConfigError(f"slots[{i}]: duplicate index {idx}")
        script_name = entry["script"]
        if not isinstance(script_name, str) or not script_name:
            raise SlotsConfigError(f"slots[{i}]: 'script' must be a non-empty string")
        script_path = scripts_dir / f"{script_name}.script"
        if not script_path.exists():
            raise SlotsConfigError(
                f"slots[{i}]: script {script_name!r} not found at {script_path}"
            )
        label = entry.get("label") or script_name
        pen_s = float(entry.get("pen_s", default_pen_s))
        out[idx] = SlotEntry(index=idx, script=script_name, label=label, pen_s=pen_s)
    return out


def default_slots_path() -> Path:
    return Path(__file__).resolve().parent.parent / "pi_slots.json"
