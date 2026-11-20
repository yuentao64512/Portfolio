"""v0.1.65 - the UAV window (8099), the OCS UAV card / tab / SYSTEM rows, the alarm rows, the launcher's UAV field, FOREIGN DECLARATION
(docs/OVERNIGHT_2026-11-07_UAV.md D15-D19). QUICK: no broker."""
from __future__ import annotations

import json
import threading
import time
import urllib.error
import urllib.request

import yaml

from optiprimex.companion.config import Config
from optiprimex.ocs.ocs_app import OcsApp
from optiprimex.tools import launcher as L
from optiprimex.tools import ocs_web as OW
from optiprimex.tools import uav_web as UW
from optiprimex.tools.window_control import Control

from .conftest import ROOT, free_port
from .test_ocs_units import FakeBus
from .test_v0164 import INSIDE, RJudge, SQ, UID

NS = "team/optiprime/v1"
NET = {"ORIN_SSH": "nx-alias", "AGX_SSH": "agx-alias"}


def env(body, vid=UID):
    return json.dumps({"vehicle_id": vid, "kind": "status", "body": body}).encode()


def uav_body(**kw):
    b = {"state": "AUTO_HOLD", "robot_state": "STATE_AUTO", "current_task": "TASK_NONE", "mode": "GUIDED", "armed": False, "lat": 1.3,
         "lon": 103.8, "altitude_hae_m": 8.0, "flight_phase": "FLIGHT_PHASE_GROUNDED", "sog_mps": 0.4, "hdg_deg": 90.0, "fcu_link": "fake",
         "health": {"telemetry_age_s": 0.5, "pilot_ready": True, "fence_ok": True, "telemetry_n": 40},
         "fence": {"points": INSIDE, "points_n": 4, "ceiling_hae_m": 20.0, "matches_declared": True, "change_pending": False},
         "uav": {"vehicle_id": UID, "registration": "UAV1", "pilot_callsign": "Calwin-U2332301", "declared": True},
         "robocommand": {"course_fp": "abc", "reports_n": 2, "last_report": {"kind": "readiness"}}}
    b.update(kw)
    return b


# ---- the UAV window ---------------------------------------------------------------------------------------------------------------
def test_the_window_model_paints_what_the_hub_says_and_nothing_more():
    clk = [100.0]
    m = UW.Model(clock=lambda: clk[0])
    s = m.snapshot()
    assert s["plate"]["link"] == "down" and s["plate"]["state"] is None and s["telemetry"]["alt_hae_m"] is None    # nothing fabricated
    m.on_message(f"{NS}/status/uav", env(uav_body()))
    m.on_message("ocs/state", json.dumps({"vehicle_id": "OCS", "kind": "telemetry", "body": {"r": "R4", "rate_dropped": 0, "uav": {
        "configured": True, "vehicle_id": UID, "declare": True, "declared": True, "declared_fence": INSIDE, "declared_ceiling_hae_m": 20.0,
        "course": {"boundary": SQ, "uav_transit_area": []}}}}).encode())
    s = m.snapshot()
    assert s["plate"] == {"link": "up", "state": "AUTO_HOLD", "mode": "GUIDED", "armed": False, "declared": True, "declare": True,
                          "pilot_ready": True, "robot_state": "STATE_AUTO", "current_task": "TASK_NONE"}
    assert s["telemetry"]["alt_hae_m"] == 8.0 and s["fence"]["points_n"] == 4 and s["fence"]["matches"] is True and s["fence"]["declared_n"] == 4
    assert s["map"]["fence"] == INSIDE and s["map"]["declared"] == INSIDE and s["map"]["course"] == SQ and s["map"]["uav"][:2] == [1.3, 103.8]
    assert s["alarms"] == []
    m.on_message(f"{NS}/status/uav", env(uav_body(health={"telemetry_age_s": 3.0, "pilot_ready": False})))
    assert m.snapshot()["plate"]["link"] == "slow" and m.snapshot()["alarms"][0]["level"] == "warn"
    m.on_message(f"{NS}/status/uav", env(uav_body(health={"telemetry_age_s": 6.0})))
    a = m.snapshot()["alarms"]
    assert a[0]["level"] == "alarm" and "UAV heartbeat stopped - Remote ID" in a[0]["text"]
    clk[0] += 10                                                                   # the adapter itself gone
    assert m.snapshot()["plate"]["link"] == "down" and "adapter gone" in m.snapshot()["alarms"][0]["text"]
    m.on_message(f"{NS}/event/uav/fence_change", json.dumps({"vehicle_id": UID, "kind": "event", "body": {"event": "fence_change", "id": "3",
                                                                                                   "vertices": INSIDE, "ceiling_hae_m": 20}}).encode())
    assert "fence change 3: 4 points" in m.snapshot()["events"][0]["text"]


def _serve(model, send):
    port = free_port()
    srv = UW.Server(("127.0.0.1", port), UW.make_handler(model, "127.0.0.1:1", Control("2468", trust_local=False), send=send))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv, f"http://127.0.0.1:{port}"


def _post(url, path, body, sid="a" * 16, hdr=True):
    h = {"Content-Type": "application/json", "X-Window-Session": sid, "X-Window-Name": "pilot"}
    if hdr:
        h["X-Uav-Web"] = "1"
    r = urllib.request.Request(url + path, data=json.dumps(body).encode(), method="POST", headers=h)
    with urllib.request.urlopen(r, timeout=5) as fh:
        return json.loads(fh.read())


def test_the_window_sends_only_the_whitelisted_lines_and_only_for_the_commander():
    sent = []
    srv, url = _serve(UW.Model(), lambda hub, line: (sent.append(line), {"ok": True, "text": "accepted " + line})[1])
    try:
        with urllib.request.urlopen(url + "/", timeout=5) as fh:
            page = fh.read().decode()
        for needle in ('id="plate"', 'id="p-ready"', 'id="map"', 'id="ctlbar"', "X-Uav-Web", "nothing here commands the drone", 'data-line="uav ready on"',
                       'data-line="declare uav on"', 'data-line="uav resend_course"', "Date.now()-ARM.t>=4000"):
            assert needle in page, needle
        r = _post(url, "/ocs", {"line": "arm"})
        assert not r["ok"] and r["text"].startswith("refused by the page: only") and not sent         # the whitelist first
        r = _post(url, "/ocs", {"line": "uav ready on"})
        assert not r["ok"] and "PIN needed" in r["text"] and not sent                                   # another computer: the PIN
        assert _post(url, "/control", {"op": "take", "pin": "2468"})["ok"]
        r = _post(url, "/ocs", {"line": "uav ready on"})
        assert r["ok"] and sent == ["uav ready on"]
        r = _post(url, "/ocs", {"line": "declare uav on"}, sid="b" * 16)
        assert not r["ok"] and "viewer" in r["text"] and sent == ["uav ready on"]
        try:
            _post(url, "/ocs", {"line": "uav ready on"}, hdr=False)
            raise AssertionError("no X-Uav-Web header must be refused")
        except urllib.error.HTTPError as e:                                                         # no header: 403
            assert e.code == 403
        with urllib.request.urlopen(url + "/state.json", timeout=5) as fh:
            st = json.loads(fh.read())
        assert st["plate"]["link"] == "down" and st["control"]["pin"] is True
        with urllib.request.urlopen(url + "/checklist.txt", timeout=5) as fh:
            assert "UAV ↔ OCS integration" in fh.read().decode()
    finally:
        srv.shutdown()
        srv.server_close()
    assert UW.ocs_line("127.0.0.1:1", "takeoff")["text"].startswith("refused by the page")
    assert set(UW.UAV_LINES) >= {"uav ready on", "uav ready off", "declare uav on", "declare uav off", "uav resend_course", "uav task 1"}
    sh = (ROOT / "scripts" / "run_uav_window.sh").read_text()
    assert 'BIND="${BIND:-${OCS_HOST:-127.0.0.1}}"' in sh and "0.0.0.0" not in sh.split("set -uo")[1] and "uavwin" in sh
    for f in ("stop_ocs.sh", "stop_bench.sh"):
        assert "run_uav_window.sh\" stop" in (ROOT / "scripts" / f).read_text(), f


# ---- the OCS page ----------------------------------------------------------------------------------------------------------------
def test_the_ocs_page_card_rows_and_alarms():
    t = OW.UavTap()
    assert t.view()["state"] == "none"
    t.on_message(json.dumps({"vehicle_id": UID, "body": uav_body()}).encode(), now=10.0)
    v = t.view(now=10.5)
    assert v["link"] == "up" and v["state"] == "AUTO_HOLD" and v["fence"]["points_n"] == 4 and "points" not in v["fence"]
    assert t.view(now=15.0)["link"] == "down"
    base = {"ocs": {"r": "R4", "rate_dropped": 0, "uav": {"configured": True, "declare": True, "declared": True}}, "uav": v}
    assert OW.uav_warnings(base) == []
    slow = dict(base, uav=dict(v, health={"telemetry_age_s": 3.1}))
    w = OW.uav_warnings(slow)
    assert len(w) == 1 and w[0]["style"] == "warn" and OW.AlarmBook.level_of(w[0]) == "CAUTION"
    stale = dict(base, uav=dict(v, health={"telemetry_age_s": 6.0}))
    w = OW.uav_warnings(stale)
    assert w[0]["style"] == "bad" and "UAV heartbeat stopped - Remote ID" in w[0]["text"] and OW.AlarmBook.level_of(w[0]) == "ALARM"
    off = {"ocs": {"r": "R2", "uav": {"configured": True, "declare": False, "declared": False}}, "uav": {"state": "none"}}
    assert OW.uav_warnings(off) == []                                                # not declared: no UAV telemetry alarm
    w = OW.uav_warnings({"ocs": {"r": "R2", "rate_dropped": 3, "foreign_declaration": {"seq": 7},
                                 "uav": {"configured": True, "last_refusal": {"reason": "the drone is ARMED"}}}, "uav": {"state": "none"}})
    texts = " | ".join(x["text"] for x in w)
    assert "DECLARATION REFUSED (UAV): the drone is ARMED" in texts and "FOREIGN DECLARATION" in texts and "RATE: 3 message(s) dropped" in texts
    page = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
    for needle in ('sysRow("UAV · adapter"', 'sysRow("UAV · window"', "function uavStep(o)", " · needs declare uav on", "pilot ready ✓",
                   "S.uav||{}", "the UAV window"):
        assert needle in page, needle
    assert 'sysRow("UAV1 · adapter (WP14)"' not in page and "not connected · unknown`;" not in page
    assert '(((S&&S.uav)||{}).link==="up")' not in page and 'UAV:"VEHICLES"' in page   # v0.1.76 (WP32): the tab is VEHICLES, never greyed


def test_foreign_declaration_and_the_uav_block_carry_the_course():
    cfg = Config()
    cfg.extra = yaml.safe_load((ROOT / "profiles" / "pool" / "ocs_pool.yaml").read_text())
    app = OcsApp(cfg, bus=FakeBus(), judge=RJudge())
    app.lifecycle.on_link(True)
    app.on_course({"course_id": "SQ", "boundary": SQ, "uav_transit_area": [], "pinger_hz": 1})
    app.on_foreign_declaration({"seq": 99, "vehicle_ids": ["UAV1"]})
    st = app.state_body()
    assert st["foreign_declaration"]["seq"] == 99 and any(a["kind"] == "foreign_declaration" for a in app.alerts)
    assert st["uav"]["course"]["boundary"] == SQ
    src = (ROOT / "optiprimex" / "ocs" / "judge_client.py").read_text()
    assert "client.subscribe(self.w.t_request(self.team_id), 1)" in src and "self.own_requests.add(seq)" in src


# ---- the launcher --------------------------------------------------------------------------------------------------------------
def test_the_launcher_uav_field():
    P = L.load_presets()
    assert all(P[k]["uav"] == "off" for k in ("sim", "lab", "event", "custom")) and L.FIELDS["uav"] == ("off", "on")
    p = L.build_plan(dict(P["sim"]), stamp="S", net_env=NET)
    assert "--uav" not in p["steps"][0]["cmd"] and not any("uav" in x["key"] for x in p["procs"])
    p = L.build_plan(dict(P["sim"], uav="on"), stamp="S", net_env=NET)
    assert p["steps"][0]["cmd"].endswith("--uav") and {"mac:optiprimex:uav", "mac:uavwin:web", "mac:optiprimex:copter"} <= {x["key"] for x in p["procs"]}
    assert p["links"]["UAV window"].endswith(":8099/")
    p = L.build_plan(dict(P["event"], uav="on"), stamp="S", net_env=NET)
    ocs = [s for s in p["steps"] if "scripts/run_ocs.sh" in s["cmd"]][0]
    assert ocs["cmd"].endswith("--uav") and {"mac:ocs:uav", "mac:uavwin:web"} <= {x["key"] for x in p["procs"]}
    p = L.build_plan(dict(P["event"], uav="on", net="manual", ocs_host="10.0.0.2", boat_host="10.0.0.3", nx_ssh="nx", agx_ssh="agx",
                          uav_host="10.0.0.9", uav_listen_port="14600"), stamp="S", net_env={})
    ocs = [s for s in p["steps"] if "scripts/run_ocs.sh" in s["cmd"]][0]
    assert "UAV_HOST=10.0.0.9" in ocs["cmd"] and "UAV_LISTEN_PORT=14600" in ocs["cmd"]
    assert L.flavour(rec={"cfg": dict(P["sim"], uav="on")})["text"].endswith(" · uav")
    assert not L.flavour(rec={"cfg": dict(P["sim"])})["text"].endswith(" · uav")
    assert any(x["name"] == "UAV window" for x in L.page_links(dict(P["sim"], uav="on"))) and not any(x["name"] == "UAV window" for x in L.page_links(P["sim"]))
    html = (ROOT / "optiprimex" / "tools" / "launcher.html").read_text()
    assert 'data-k="uav"' in html and 'data-k="uav_host"' in html and "uav:{off:" in html
    assert {"uav_host", "uav_listen_port", "uav_bridge_port", "uav_broker_host", "uav_broker_port", "uav_buoy_path"} <= set(L.MANUAL_KEYS)


def test_quick_list_has_this_file():
    assert "tests/test_v0165.py" in (ROOT / "Makefile").read_text()
