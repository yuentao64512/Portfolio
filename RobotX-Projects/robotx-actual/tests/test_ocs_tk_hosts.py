"""WP44 (K8): the tk console's HOSTS tab - AGX · NX · MAC from GET /hosts.json (session B's v0.1.86 contract, PROMPT_PRESETS_NIGHT §2.5
+ §6: disk_free_gb, rec_gb_per_10min). The fixture follows the contract; the model without Tk, then the tab in the window."""
from __future__ import annotations

import json
from pathlib import Path

import pytest

from optiprimex.tools.tkconsole import hosts as H

FIX = Path(__file__).parent / "fixtures"


def body():
    return json.loads((FIX / "hosts_sample.json").read_text())


def test_three_cards_in_order_with_their_numbers():
    cards = H.host_cards(body())
    assert [c.key for c in cards] == ["agx", "nx", "mac"]
    agx = cards[0]
    m = {x.key: x for x in agx.metrics}
    assert agx.title == "AGX · optiprime-agxorin" and agx.sub == "MAXN · 0.7 s" and not agx.stale and agx.throttle == ""
    assert m["cpu"].value == "63 %" and m["gpu"].value == "88 %" and m["temp"].value == "71.5 °C" and m["temp"].note == "power 32.4 W"
    assert m["ram"].value == "21.0 / 29.9 GB · 70 %" and len(m["cpu"].hist) == 600 and m["temp"].hist2
    assert agx.disk == "disk free 12.4 GB · ≈ 78 min of recording left at 1.6 GB / 10 min" and agx.disk_level == "warn"   # < 15 GB


def test_unknowns_are_dashes_never_a_crash():
    mac = H.host_cards(body())[2]
    m = {x.key: x for x in mac.metrics}
    assert m["gpu"].value == "—" and m["temp"].value == "—" and m["temp"].note == "" and mac.sub == "0.1 s"
    assert mac.disk == "disk free 43.0 GB · recording rate not measured" and mac.disk_level == ""
    assert H.host_cards({"hosts": {"agx": {}}})[0].disk == "disk free —" and H.host_cards(None) == []


def test_levels_stale_throttling_and_sustained_cpu():
    b = body()
    a = b["hosts"]["agx"]
    a.update(age_s=9.0, throttling=True, throttle_src="thermal")
    a["last"].update(cpu_pct=97.0, gpu_pct=99.0, temp_max_c=86.0, disk_free_gb=6.0)
    a["hist"]["cpu_pct"][-10:] = [97.0] * 10                       # 10 s over 90 %: not yet red
    agx = H.host_cards(b)[0]
    m = {x.key: x for x in agx.metrics}
    assert agx.stale and agx.sub == "MAXN · no beat" and agx.throttle == "THROTTLING · thermal"
    assert m["cpu"].level == "warn" and m["gpu"].level == "bad" and m["temp"].level == "bad" and agx.disk_level == "bad"
    a["hist"]["cpu_pct"][-31:] = [97.0] * 31                       # 31 s over 90 %: red
    assert {x.key: x for x in H.host_cards(b)[0].metrics}["cpu"].level == "bad"
    a["last"]["temp_max_c"] = 81.0
    assert {x.key: x for x in H.host_cards(b)[0].metrics}["temp"].level == "warn"       # amber from 80


tk = pytest.importorskip("tkinter")
from .test_ocs_tk_smoke import shared_root, spin  # noqa: E402
from .test_ocs_tk_net import FakeOcs  # noqa: E402


@pytest.fixture()
def root():
    top = tk.Toplevel(shared_root())
    top.withdraw()
    top.quit, top.mainloop = shared_root().quit, shared_root().mainloop
    yield top
    try:
        top.destroy()
    except tk.TclError:
        pass


def test_the_hosts_tab_draws_three_cards_and_says_when_the_backend_is_old(root):
    from optiprimex.tools.tkconsole.views import App
    f = FakeOcs("r5_task")
    app = App(root, f.url, fast_hz=5)
    try:
        assert spin(root, lambda: app.view is not None and app.view.link == "UP")
        app.nb.select(app.tabs["HOSTS"])
        assert spin(root, lambda: set(app.hosts.cards) == {"agx", "nx", "mac"}, 5)
        agx = app.hosts.cards["agx"]
        assert agx["rows"]["cpu"]["cv"].find_withtag("trace") and agx["rows"]["temp"]["cv"].find_withtag("trace2")
        assert agx["disk"].cget("text").startswith("disk free 12.4 GB · ≈ 78 min")
        assert app.hosts.cards["mac"]["rows"]["gpu"]["val"].cget("text") == "—"
        assert "/hosts.json" in f.gets
        del f.bodies["/hosts.json"]                                 # an OCS before B's v0.1.86: 404
        app.hosts.cards.clear()
        assert spin(root, lambda: app.hosts.note.cget("text") == H.NEEDS, 5)
        app.nb.select(app.tabs["LOG"])
        assert spin(root, lambda: not app.poller.hosts_on.is_set(), 2)    # polled only while the tab is visible
    finally:
        app.close()
        f.close()


def test_the_signal_bar_measured_and_estimated():
    assert H.link_quality({"rtt_ms": 8, "loss_pct": 0, "iface": "eth0"}) == (4, "excellent · 8 ms · 0 % loss · eth0", "ok")
    assert H.link_quality({"rtt_ms": 45, "loss_pct": 1, "rssi_dbm": -62})[0] == 3
    assert H.link_quality({"rtt_ms": 120, "loss_pct": 6})[:3:2] == (2, "warn")
    assert H.link_quality({"rtt_ms": 30, "loss_pct": 0, "rssi_dbm": -85})[0] == 1                 # weak Wi-Fi caps it
    b, t, l = H.link_quality(None, {"age_s": 0.4, "up": True, "what": "boat status"})
    assert b == 4 and t.startswith("excellent · estimated · boat status 0.4 s old") and l == "ok"
    assert H.link_quality(None, {"age_s": None, "up": False, "what": "boat status"})[:3:2] == (0, "bad")
    assert H.link_quality(None, None) == (0, "link — (not measured)", "")
    cards = {c.key: c for c in H.host_cards(body(), {"nx": {"age_s": 0.4, "up": True, "what": "boat status"}})}
    assert cards["nx"].bars == 4 and cards["mac"].link == "this Mac · local" and cards["agx"].bars == 0
