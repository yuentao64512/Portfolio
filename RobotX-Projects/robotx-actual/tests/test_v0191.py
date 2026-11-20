"""v0.1.91 (Dmitrii 2026-11-12): led_core - the horizon cut, the LED colour under the hat without a model, the blink memory without a
map (steady vs flashing, Task 1's blue exit vs entry). Checked on 12 Nov's 629 detections: the horizon at 0.45 x the height drops the 6
sky / building boxes and nothing with depth; the LED reader at v_min 215 triggers once - the blue lantern on marker 1 at 7 m."""
from __future__ import annotations

import importlib.util

import numpy as np

from .conftest import ROOT


def _load():
    spec = importlib.util.spec_from_file_location("led_core_v0191", ROOT / "cv" / "buoys_multi" / "led_core.py")
    m = importlib.util.module_from_spec(spec)
    import sys
    sys.modules[spec.name] = m                                         # the dataclass needs its module registered
    spec.loader.exec_module(m)
    return m


LC = _load()
BOX = [100, 100, 200, 260]                      # a buoy box; v0.1.93.2: the LED band = -45..+5 % of its height (the lantern above the box), the middle 40 %


def frame(led=None):
    """A grey water / white buoy frame; led = 'red' | 'green' | 'blue' paints a bright saturated blob in the LED band."""
    im = np.full((400, 400, 3), (60, 90, 70), np.uint8)
    im[100:260, 100:200] = (235, 235, 235)
    if led:
        bgr = {"red": (20, 20, 255), "green": (20, 255, 20), "blue": (255, 40, 20)}[led]
        im[50:80, 140:160] = bgr
    return im


def test_led_colour_without_a_model():
    for c in ("red", "green", "blue"):
        colour, on, shares = LC.led_sample(frame(c), BOX)
        assert on and colour == c and shares[c] > 0.02, (c, shares)
    colour, on, _ = LC.led_sample(frame(None), BOX)
    assert not on and colour is None                                   # a white buoy, no LED
    pale = frame(None)
    pale[50:80, 140:160] = (205, 190, 150)                           # the pale blue float (V ~205): not an LED
    assert LC.led_sample(pale, BOX)[1] is False


def test_the_blink_memory_tells_steady_from_flashing():
    c = LC.LedCache()
    t, out = 0.0, None
    for k in range(20):                                                # 5 fps, a 1 Hz blink: on for 0.5 s, off for 0.5 s
        on = (k // 3) % 2 == 0
        out = c.update(t, [(BOX, on, "blue" if on else None)])
        t += 0.2
    tid, pat = out[0]
    assert pat["pattern"] == "flashing" and pat["colour"] == "blue" and pat["changes"] >= 2      # Task 1's ENTRY
    s = LC.LedCache()
    for k in range(20):
        out = s.update(k * 0.2, [(BOX, True, "blue")])
    assert out[0][1]["pattern"] == "steady" and out[0][1]["colour"] == "blue"                     # Task 1's EXIT
    o = LC.LedCache()
    for k in range(20):
        out = o.update(k * 0.2, [(BOX, False, None)])
    assert out[0][1]["pattern"] == "off"
    assert LC.LedCache().update(0.0, [(BOX, True, "red")])[0][1]["pattern"] == "unknown"           # too few samples yet


V091 = {"rule": "v091", "window_s": 4.0, "min_samples": 6, "steady_frac": 0.85}     # v0.1.92 made "runs" the default; this pins v0.1.91's


def test_two_buoys_keep_their_own_tracks_and_a_lost_one_is_forgotten():
    c = LC.LedCache(V091)
    far = [300, 100, 360, 200]
    for k in range(10):
        out = c.update(k * 0.2, [(BOX, True, "red"), (far, k % 2 == 0, "green" if k % 2 == 0 else None)])
    (t1, p1), (t2, p2) = out
    assert t1 != t2 and p1["colour"] == "red" and p1["pattern"] == "steady" and p2["pattern"] == "flashing"
    out = c.update(10.0, [(BOX, True, "red")])                          # 8 s later: the old tracks are gone, a new one starts
    assert out[0][0] not in (t1, t2) and out[0][1]["n"] == 1


def test_the_horizon_drops_boxes_above_it():
    hy = LC.horizon_y(540)                                              # no IMU: 0.45 x the height
    assert hy == 243.0
    assert LC.above_horizon([717, 0, 960, 127], hy)                     # 12 Nov's building box (16:55:53)
    assert not LC.above_horizon([310, 319, 422, 401], hy)               # a marker at 7 m
    assert LC.horizon_y(1242, fy=2293.0, cy=621.0, pitch_deg=0.0) == 621.0
    assert LC.horizon_y(1242, fy=2293.0, cy=621.0, pitch_deg=2.0) > 621.0      # nose up: the horizon moves down in the image


def test_the_observer_applies_both_to_every_detector():
    src = (ROOT / "cv" / "buoys_multi" / "multi_buoy_observer.py").read_text()
    assert 'p.add_argument("--horizon", choices=("frac", "off"), default="frac"' in src and '"--horizon-frac", type=float, default=0.45' in src
    assert "if hy is not None and LC.above_horizon(box.xyxy[0].cpu().numpy().tolist(), hy):" in src
    assert 'p.add_argument("--led", choices=("on", "off"), default="on"' in src and 'b["led"] = dict(pat, on=on, now=colour)' in src
    assert src.index("led_cache.update(") < src.index("payload = M.frame_payload(")              # in latest_status.json + frames.jsonl
    assert '"led"' in (ROOT / "cv" / "buoys_multi" / "rec_core.py").read_text()
