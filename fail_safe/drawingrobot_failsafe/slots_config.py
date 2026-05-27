"""Loader for `pi_slots.json`: maps `/robot/mode_cmd` Int8 codes 80..89 to
scripts in `fail_safe/scripts/<name>.script`.

Schema is identical to the parent's so the same JSON can drop in. The
parent's `pen_s` field is accepted-and-ignored here: in fail-safe the
pen is fixed at the left wheel, so per-slot pen positions don't apply.

    {
      "defaults": { "pen_s": 0.0 },
      "slots": [
        {"index": 0, "script": "square", "label": "Square"},
        {"index": 1, "script": "circle"}
      ]
    }

Validation: index in [0, 9], unique across slots, script must resolve to
a file on disk under `fail_safe/scripts/`.
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
        out[idx] = SlotEntry(index=idx, script=script_name, label=label)
    return out


def default_slots_path() -> Path:
    return Path(__file__).resolve().parent.parent / "pi_slots.json"
