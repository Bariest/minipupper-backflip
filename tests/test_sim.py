"""The sim, the recordings and the fit still agree with the numbers quoted in the docs."""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from backflip import FITTED, Floor, Preset, simulate  # noqa: E402
from backflip import recording  # noqa: E402
from backflip.sysid import Fitter  # noqa: E402


def test_k7_405_flips_in_sim():
    m = simulate(Preset.load("k7_405")).metrics
    assert 240 < m["rot_td"] < 265, m["rot_td"]           # docs: 252 deg
    assert m["upright"]
    assert 150 < m["flight_ms"] < 250


def test_recordings_measure():
    m = recording.load("flip_81").metrics
    assert 265 < m["rot_td"] < 285 and m["upright"]
    assert recording.load("flip_112").metrics["upright"] is False   # grass: fell


def test_fitted_model_beats_nominal():
    k7, o1 = Preset.load("k7_405"), Preset.load("o1")
    recs = [(recording.load("flip_81"), k7, 1.0), (recording.load("flip_84"), o1, 1.0)]
    f = Fitter(recs)
    fitted, _ = f.score((FITTED, Floor(0.7)))
    nominal, _ = f.score((FITTED.__class__(1.0, 1.0, 1.0, 1.0, 5.0), Floor(1.0)))
    assert fitted < 0.5 * nominal, (fitted, nominal)


def test_presets_fit_a_centred_stand():
    for p in sorted((ROOT / "presets").glob("*.json")):
        assert not Preset.load(p).rail_check([511] * 12), p.name
