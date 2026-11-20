"""v0.1.94.3 (Dmitrii 14 Nov): the live speed knobs - cruise / circle / gate x0.5-2.0, any state, the cap x k up to 1.5 m/s."""
from pathlib import Path

from optiprimex.companion import speed_k as SK
from optiprimex.companion.config import Config
from optiprimex.companion.state import GuidedTarget, VelocityYawTarget
from optiprimex.usv.behaviours.base import BehaviourOut
from optiprimex.usv.behaviours.follow_path import FollowPath
from optiprimex.usv.behaviours.orbit import Orbit
from optiprimex.usv.behaviours.scan_yaw import ScanYaw
from optiprimex.usv.tasks import base as TB
ROOT = Path(__file__).resolve().parents[1]


def test_check_range_and_words():
    assert SK.check({"cruise": "1.25"}) == ("", {"cruise": 1.25})
    assert SK.check({"all": 2})[1] == {"cruise": 2.0, "circle": 2.0, "gate": 2.0}
    assert SK.check({"reset": True})[1] == {"cruise": 1.0, "circle": 1.0, "gate": 1.0}
    assert SK.check({"cruise": 2.25})[0] and SK.check({"cruise": 0.25})[0] and SK.check({"warp": 1})[0] and SK.check({"gate": "x"})[0]
    ex = {}
    SK.apply(ex, {"gate": 1.5})
    assert SK.current(ex) == {"cruise": 1.0, "circle": 1.0, "gate": 1.5}


def test_classes_by_behaviour_and_t1_gate():
    from optiprimex.usv.tasks import t1_safe_passage as T1
    t1 = next(v for v in vars(T1).values() if isinstance(v, type) and getattr(v, "SPEED_CLASS", None))
    assert TB.speed_class(t1, "THROUGH", FollowPath()) == "gate"
    assert TB.speed_class(t1, "TRANSIT", FollowPath()) == "cruise"
    assert TB.speed_class(t1, "CIRCLE_ENTRY_CW", Orbit()) == "circle"
    assert TB.speed_class(t1, "SCAN", ScanYaw()) is None


def test_scale_and_cap():
    cfg = Config()
    cfg.limits.max_speed_mps = 0.6
    out = BehaviourOut(cmd=VelocityYawTarget(0.4, 0.1))
    TB.scale_speed(out, "circle", 2.0)
    TB.cap_speed(out, cfg, 2.0)
    assert abs(out.cmd.vx_mps - 0.8) < 1e-9 and abs(out.cmd.yaw_rate_rad_s - 0.2) < 1e-9     # the cap lifted to 1.2; the radius kept
    out = BehaviourOut(cmd=VelocityYawTarget(0.4, 0.1))
    TB.scale_speed(out, "cruise", 2.0)
    assert abs(out.cmd.yaw_rate_rad_s - 0.1) < 1e-9                                       # cruise: heading control untouched
    cfg.limits.max_speed_mps = 1.0
    out = BehaviourOut(cmd=GuidedTarget(1.0, 2.0, 0.9))
    TB.scale_speed(out, "cruise", 2.0)
    TB.cap_speed(out, cfg, 2.0)
    assert out.cmd.speed_mps == SK.V_CEIL                                                 # never above 1.5 m/s
    cfg.limits.max_speed_mps = 0.6
    out = BehaviourOut(cmd=VelocityYawTarget(0.6, 0.0))
    TB.scale_speed(out, "gate", 0.5)
    TB.cap_speed(out, cfg, 0.5)
    assert abs(out.cmd.vx_mps - 0.3) < 1e-9


def test_the_word_end_to_end():
    from optiprimex.ocs import cmd_authority as CA
    from optiprimex.tools import ocs_cli, playbook, bench_tui
    from optiprimex.usv.command_handler import CommandHandler
    from optiprimex.usv.run_context import RUN_FREE_ACTIONS
    p = ocs_cli.build_parser()
    assert ocs_cli.intent_spec(p.parse_args(["speed", "cruise", "1.25"]))[:3] == ("SPEED", None, {"cruise": "1.25"})
    assert ocs_cli.intent_spec(p.parse_args(["speed", "reset"]))[2] == {"reset": True}
    assert CA.GUARDS["SPEED"][2] == "SET_SPEED_K" and "SET_SPEED_K" in RUN_FREE_ACTIONS
    assert "SPEED" in playbook.PLAYBOOK and "speed" in bench_tui.INTENT_WORDS
    assert '"SET_SPEED_K"' in (ROOT / "optiprimex" / "schemas" / "bodies.schema.json").read_text()
    h = CommandHandler.__new__(CommandHandler)
    h.cfg, h.on_tuned = Config(), None
    r = h._speed({"circle": 1.5})
    assert r[0] == "ACCEPTED" and SK.k_of(h.cfg, "circle") == 1.5 and SK.k_of(h.cfg, "cruise") == 1.0
    assert h._speed({"circle": 3})[0] == "REJECTED"
