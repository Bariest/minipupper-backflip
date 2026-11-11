"""The C firmware player and the Python player must produce the same servo targets, tick for tick."""
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from backflip import Preset  # noqa: E402

HOST = ROOT / "firmware" / "host"
PRESETS = sorted(p.stem for p in (ROOT / "presets").glob("*.json"))


@pytest.fixture(scope="module")
def play_table():
    if not (shutil.which("cc") or shutil.which("gcc") or shutil.which("clang")):
        pytest.skip("no C compiler")
    subprocess.run(["make", "-C", str(HOST), "-B"], check=True, capture_output=True)
    return HOST / "play_table"


@pytest.mark.parametrize("name", PRESETS)
def test_c_player_matches_python(play_table, name):
    out = subprocess.run([str(play_table), name], check=True, capture_output=True, text=True).stdout
    c_ticks = np.array([[int(v) for v in ln.split(",")] for ln in out.strip().splitlines()])
    p = Preset.load(name)
    py_ticks, arms = p.play(stand_ms=300, hold_end_ms=600)
    py_ticks = py_ticks[arms[0]:]                     # drop the stand rows before frame 0
    assert c_ticks.shape == py_ticks.shape
    assert np.array_equal(c_ticks, py_ticks.astype(int)), np.argwhere(c_ticks != py_ticks)[:5]


def test_rail_check_refuses():
    p = Preset.load("k7_405")
    stand = [511] * 12
    stand[2] = 30                                     # front-right knee stands near its end
    assert p.rail_check(stand), "a table that would pass the end stop must be reported"
    assert not p.rail_check([511] * 12), "an all-centred stand fits every frame"
