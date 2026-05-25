import json
from pathlib import Path

import pytest

from drawingrobot.slots_config import (
    SlotsConfigError,
    default_slots_path,
    load_slots,
    parse_slots,
)


@pytest.fixture
def fake_scripts(tmp_path: Path) -> Path:
    sd = tmp_path / "scripts"
    sd.mkdir()
    for name in ("alpha", "beta", "gamma"):
        (sd / f"{name}.script").write_text("forward 10\n")
    return sd


def test_repo_slot_config_loads():
    # The checked-in pi_slots.json must always be parseable; otherwise
    # the sim and pi-service blow up on launch.
    slots = load_slots(default_slots_path())
    assert len(slots) >= 1
    for idx, entry in slots.items():
        assert 0 <= idx <= 9
        assert entry.index == idx
        assert entry.script


def test_parse_valid(fake_scripts: Path):
    raw = {
        "defaults": {"pen_s": 0.25},
        "slots": [
            {"index": 0, "script": "alpha", "label": "First"},
            {"index": 3, "script": "beta", "pen_s": 0.5},
        ],
    }
    slots = parse_slots(raw, scripts_dir=fake_scripts)
    assert set(slots) == {0, 3}
    assert slots[0].label == "First"
    assert slots[0].pen_s == 0.25       # inherited from defaults
    assert slots[3].label == "beta"     # falls back to script name
    assert slots[3].pen_s == 0.5        # per-slot override


@pytest.mark.parametrize("bad_index", [-1, 10, 999])
def test_rejects_out_of_range_index(fake_scripts: Path, bad_index: int):
    raw = {"slots": [{"index": bad_index, "script": "alpha"}]}
    with pytest.raises(SlotsConfigError, match="index"):
        parse_slots(raw, scripts_dir=fake_scripts)


def test_rejects_duplicate_index(fake_scripts: Path):
    raw = {
        "slots": [
            {"index": 2, "script": "alpha"},
            {"index": 2, "script": "beta"},
        ],
    }
    with pytest.raises(SlotsConfigError, match="duplicate"):
        parse_slots(raw, scripts_dir=fake_scripts)


def test_rejects_missing_script_file(fake_scripts: Path):
    raw = {"slots": [{"index": 0, "script": "nope"}]}
    with pytest.raises(SlotsConfigError, match="not found"):
        parse_slots(raw, scripts_dir=fake_scripts)


def test_rejects_missing_required_fields(fake_scripts: Path):
    with pytest.raises(SlotsConfigError, match="missing 'script'"):
        parse_slots({"slots": [{"index": 0}]}, scripts_dir=fake_scripts)
    with pytest.raises(SlotsConfigError, match="missing 'index'"):
        parse_slots({"slots": [{"script": "alpha"}]}, scripts_dir=fake_scripts)


def test_rejects_non_list_slots(fake_scripts: Path):
    with pytest.raises(SlotsConfigError, match="slots"):
        parse_slots({"slots": "alpha"}, scripts_dir=fake_scripts)


def test_load_slots_missing_file(tmp_path: Path):
    with pytest.raises(SlotsConfigError, match="not found"):
        load_slots(tmp_path / "no_such.json")


def test_load_slots_invalid_json(tmp_path: Path):
    p = tmp_path / "bad.json"
    p.write_text("{not json}")
    with pytest.raises(SlotsConfigError, match="invalid JSON"):
        load_slots(p)
