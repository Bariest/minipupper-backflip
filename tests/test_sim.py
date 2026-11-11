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


def test_landing_metrics_real():
    assert recording.load("flip_115").metrics["overshoot"] > 30       # docs/07: sat back 47 deg
    assert recording.load("flip_81").metrics["overshoot"] < 20        # the weaker flips: 8-15 deg


def test_l2_keeps_the_push():
    k7, l2 = Preset.load("k7_405"), Preset.load("k7_405_l2")
    assert l2.frames[:4] == k7.frames[:4] and l2.move_ms[:4] == k7.move_ms[:4] and l2.hold_ms[:4] == k7.hold_ms[:4]
    from backflip import ENSEMBLE, ENSEMBLE_FLOORS
    like_today = (ENSEMBLE[2].at(12.5), ENSEMBLE_FLOORS[2])           # the fitted model that matches flips 115-116
    before, after = simulate(k7, *like_today).metrics, simulate(l2, *like_today).metrics
    assert before["overshoot"] > 30 and after["overshoot"] < 10 and after["upright"]
