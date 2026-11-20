"""v0.1.94 (Dmitrii 2026-11-14: "can we make the course appearing regardless of port we plug in?"): scripts/net_roles.py's rule on
made-up documentation addresses (RFC 5737) - the team side gets OCS_HOST, the course side keeps its DHCP lease, whichever cable is where."""
from __future__ import annotations

import importlib.util

from .conftest import ROOT

spec = importlib.util.spec_from_file_location("net_roles", ROOT / "scripts" / "net_roles.py")
NR = importlib.util.module_from_spec(spec)
spec.loader.exec_module(NR)

CONF = NR.conf_from_env({"OCS_HOST": "192.0.2.10", "RC_CHARLIE_HOST": "198.51.100.5", "RC_ALPHA_HOST": "203.0.113.5"})
OCS = "192.0.2.10"


def ifs(**kw):
    return {d: {"active": a is not None, "addrs": a or []} for d, a in kw.items()}


def test_conf_from_env():
    assert CONF["team_net"] == "192.0.2.0/24" and CONF["courses"] == {"CHARLIE": "198.51.100.0/24", "ALPHA": "203.0.113.0/24"}


def test_either_cable_layout_gives_the_same_roles():
    for course, team in (("en19", "en20"), ("en20", "en19")):
        roles, team_dev, acts = NR.plan(ifs(**{course: ["198.51.100.77"], team: ["192.0.2.55"]}), CONF, {}, 100.0)
        assert roles[course] == "course:CHARLIE" and team_dev == team and acts == [("add", team)]


def test_13_nov_the_alias_left_on_the_course_side_moves():
    roles, team_dev, acts = NR.plan(ifs(en20=["198.51.100.77", OCS], en19=["192.0.2.55"]), CONF, {}, 100.0)
    assert roles["en20"] == "course:CHARLIE" and team_dev == "en19" and acts == [("del", "en20"), ("add", "en19")]


def test_nothing_to_do_when_right_and_both_on_the_switch_takes_one():
    assert NR.plan(ifs(en19=["192.0.2.55", OCS], en20=["198.51.100.77"]), CONF, {}, 0.0)[2] == []
    roles, team_dev, acts = NR.plan(ifs(en19=["192.0.2.55"], en20=["192.0.2.56"]), CONF, {}, 0.0)
    assert team_dev in ("en19", "en20") and len(acts) == 1


def test_a_switch_without_dhcp_after_the_wait_never_a_cable_still_leasing():
    lay = ifs(en19=[], en20=["198.51.100.77"])
    lay["en19"]["active"] = True
    assert NR.plan(lay, CONF, {"en19": 95.0}, 100.0)[1] is None                  # 5 s: the lease may still come
    roles, team_dev, acts = NR.plan(lay, CONF, {"en19": 70.0}, 100.0)
    assert team_dev == "en19" and roles["en19"] == "team?" and acts == [("add", "en19")]
    lay2 = ifs(en19=[], en20=[])
    lay2["en19"]["active"] = lay2["en20"]["active"] = True
    assert NR.plan(lay2, CONF, {"en19": 0.0, "en20": 0.0}, 100.0)[1] is None    # two bare adapters: no guess


def test_unplugged_adapter_loses_nothing_and_wifi_is_never_listed():
    roles, team_dev, acts = NR.plan(ifs(en19=None), CONF, {}, 0.0)
    assert roles == {"en19": "down"} and team_dev is None and acts == []
    src = (ROOT / "scripts" / "net_roles.py").read_text()
    assert "Wi-Fi" in src and "print(conf" not in src and "ocs_host\"]}" not in src     # never prints the addresses


def test_judge_auto_never_picks_the_venue_demo():
    """v0.1.94.2 (Dmitrii: "judge auto should not connect to venue demo")."""
    conf = NR.conf_from_env({"OCS_HOST": "192.0.2.10", "RC_CHARLIE_HOST": "198.51.100.5", "VENUE_RC_HOST": "203.0.113.9"})
    roles, _, _ = NR.plan(ifs(en19=["203.0.113.77"], en20=["192.0.2.55"]), conf, {}, 0.0)
    assert roles["en19"] == "course:VENUE" and NR.judge_course(roles) is None
    roles, _, _ = NR.plan(ifs(en19=["203.0.113.77"], en20=["198.51.100.77"]), conf, {}, 0.0)
    assert NR.judge_course(roles) == "CHARLIE"


def test_a_lease_from_another_network_never_takes_the_team_fallback():
    """v0.1.94.2: an iPhone USB tether (or a home router) leases an address outside the team / course nets - not a bare switch port."""
    since = {"en19": 0.0, "en7": 0.0}
    roles, team_dev, acts = NR.plan(ifs(en19=["169.254.3.4"], en7=["172.20.10.2"]), CONF, since, 100.0)
    assert roles["en7"] == "other" and team_dev == "en19" and acts == [("add", "en19")]
    roles, team_dev, acts = NR.plan(ifs(en7=["172.20.10.2"]), CONF, {"en7": 0.0}, 100.0)
    assert team_dev is None and acts == []


def test_moving_the_switch_cable_to_the_other_adapter_moves_the_alias():
    roles, team_dev, acts = NR.plan(ifs(en20=None, en19=["192.0.2.55"]), CONF, {}, 100.0)
    assert team_dev == "en19" and acts == [("add", "en19")]
    roles, team_dev, acts = NR.plan({"en20": {"active": False, "addrs": [OCS]}, "en19": {"active": True, "addrs": ["192.0.2.55"]}}, CONF, {}, 100.0)
    assert team_dev == "en19" and ("del", "en20") in acts and ("add", "en19") in acts
