"""rctest_manual: RoboNation's rc-test server (rc_test/test_server.py, unmodified) with its auto-RunStart off (v0.1.52) and our course (v0.1.53).

RoboNation's test_server.py publishes RunStart by itself the moment every declared vehicle reports STATE_AUTO (its `start_run`,
called from `handle_heartbeat`); there is no switch for it. Dmitrii wants to press RunStart himself on the judge page (as the
real course manager approves a run, GUIDE 4.17c). This runner imports the server file as it is, replaces only `start_run` with a
log line ("[manual-start] ... waiting for the judge's RunStart") and runs its `main()`: the course, the declaration checks, the
logging (the Proof-of-Readiness log) stay RoboNation's. The RunStart then comes from the judge page (8095, --as-judge) or
rc-tui's `start`, which publish a real RxCommand on the same broker.

  cd <robocommand>/RobotX_2026/rc_test && BROKER_PORT=1885 LOG_DIR=... PYTHONPATH=../gen/python:<kit> python -m optiprimex.tools.rctest_manual test_server.py
  scripts/run_rctest.sh starts it (RCTEST_RUNSTART=manual, the default; RCTEST_RUNSTART=auto = test_server.py itself, as before).

v0.1.53, the course: RCTEST_COURSE=extra (default) | alpha - the server's module constants COURSE_ALPHA / TRANSIT_A (the RxCourse it
publishes at start AND the rings its UAV-geofence check uses) are set to that catalogue course (sim/courses.py; alpha = RoboNation's
own values, unchanged), COURSE_ID / PINGER_FREQ_HZ (its own environment switches) to its id and pinger unless already set. And it
follows a course re-published on its broker (the judge page's "publish course", --as-judge): a retained RxCourse with other corners
replaces the rings, so the geofence check stays on the course the OCS was given (RCTEST_FOLLOW=0: off). With RCTEST_RUNSTART=auto
and a course other than alpha run_rctest.sh uses this runner too (RCTEST_RUNSTART=auto here = keep the server's auto-RunStart).
"""
from __future__ import annotations

import importlib.util
import os
import sys
from types import ModuleType

MANUAL_LINE = "[manual-start] All declared vehicles autonomous for {team} — RunStart is manual: press RunStart on the judge page " \
              "(or rc-tui `start {seq}`) (declaration_seq={seq})"


def load(path: str) -> ModuleType:
    """Import RoboNation's server file as the module `test_server` (its own directory first on sys.path, as `python test_server.py`)."""
    path = os.path.abspath(path)
    sys.path.insert(0, os.path.dirname(path))
    spec = importlib.util.spec_from_file_location("test_server", path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["test_server"] = mod
    spec.loader.exec_module(mod)
    return mod


def make_manual(mod: ModuleType) -> ModuleType:
    """Replace the server's auto-RunStart by a log line, once per declaration (as the original marks `started`)."""
    def start_run(client, team_id, pending):            # noqa: ARG001 - the server's own signature
        pending["started"] = True
        mod.log.info(MANUAL_LINE.format(team=team_id, seq=pending.get("seq")))
    mod.start_run = start_run
    mod.RUNSTART_MODE = "manual"
    return mod


def apply_course(mod: ModuleType, course_id: str = "extra") -> str:
    """Point the server at a catalogue course: its COURSE_ALPHA / TRANSIT_A (closed rings, as RoboNation's) and the COURSE_ID /
    PINGER_FREQ_HZ environment its main() reads (only when not set already). Returns the course id; unknown ids raise ValueError."""
    from ..sim import courses as C
    crs = C.get(course_id)
    if crs is None:
        raise ValueError(f"RCTEST_COURSE={course_id!r}: one of {', '.join(c.lower() for c in C.COURSES)}")
    mod.COURSE_ALPHA = [tuple(p) for p in crs.ring()]
    mod.TRANSIT_A = [tuple(p) for p in crs.transit_ring()]
    os.environ.setdefault("COURSE_ID", crs.id)
    os.environ.setdefault("PINGER_FREQ_HZ", str(int(crs.pinger_hz)))
    mod.RCTEST_COURSE = crs.id
    return crs.id


def follow_courses(mod: ModuleType) -> ModuleType:
    """Wrap the server's on_message: an RxCourse on its broker with other corners than its rings (the judge page republished the
    course) becomes its COURSE_ALPHA / TRANSIT_A - logged as `[course] following ...` - before RoboNation's handler logs it as usual."""
    orig = mod.on_message

    def on_message(client, userdata, msg):
        if str(getattr(msg, "topic", "")).endswith("/course"):
            try:
                c = mod.rx_course_pb2.RxCourse()
                c.ParseFromString(msg.payload)
                ring = [(p.latitude, p.longitude) for p in c.corners]
                if len(ring) >= 4 and ring != list(mod.COURSE_ALPHA):
                    mod.COURSE_ALPHA = ring
                    mod.TRANSIT_A = [(p.latitude, p.longitude) for p in c.uav_transit_area]
                    mod.RCTEST_COURSE = c.course_id
                    mod.log.info(f"[course] following RxCourse {c.course_id} ({len(ring)} points): the geofence check uses it now")
            except Exception as e:                       # never stop the server over our wrapper
                mod.log.warning(f"[course] could not follow {msg.topic}: {e}")
        return orig(client, userdata, msg)
    mod.on_message = on_message
    return mod


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    path = argv[0] if argv else "test_server.py"
    if not os.path.isfile(path):
        print(f"rctest_manual: {path} not found (run it in <robocommand>/RobotX_2026/rc_test)", file=sys.stderr)
        return 2
    mod = load(path)
    if os.environ.get("RCTEST_RUNSTART", "manual") != "auto":
        make_manual(mod)
        mod.log.info("[manual-start] rctest_manual: the auto-RunStart is off - the judge page's RunStart starts the run")
    if hasattr(mod, "COURSE_ALPHA"):                      # v0.1.53: the course (a stand-in server without one: left alone)
        cid = apply_course(mod, os.environ.get("RCTEST_COURSE", "extra"))
        if hasattr(mod, "on_message") and os.environ.get("RCTEST_FOLLOW", "1") != "0":
            follow_courses(mod)
        mod.log.info(f"[course] rctest_manual: RxCourse {os.environ.get('COURSE_ID', cid)} = the catalogue's {cid} (RCTEST_COURSE; "
                     "a course re-published on this broker is followed)")
    mod.main()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
