"""uav_gate (v0.1.89, Dmitrii 2026-11-12): the drone's touchdown as a micro-state sequence - when may the boat leave WAIT_UAV_MAP.

"when the drone lands it goes disarm, we can account on that as the micro state sequence - map quiet, Guided (RTL) to disarm, after that
we can issue command to boat" + "Automatic": the boat goes by itself once the touchdown is confirmed.

The sequence (each step latched once seen, shown on the OCS ladder and the run banner):
  1 FLYING     the drone seen armed or airborne (the UAV adapter's `flown` latches, forwarded by the OCS as uav_ocs.flight)
  2 MAP QUIET  no new / moved / recoloured UAV buoy for quiet_s - their publisher repeats the WHOLE map ~1 Hz while their CV runs, so
               the map's change time (PerceptionMap.uav_change_t), not its message time
  3 RTL/LAND   the drone's mode seen RTL / SMART_RTL / LAND / AUTO_RTL - informational: a pilot may land by hand in LOITER
  4 DISARMED   armed False after FLYING - the touchdown
GO = FLYING and DISARMED and MAP QUIET (the operator's `uav scan done` and the UAV's scan.complete still go at once). Pure: no I/O.
"""
from __future__ import annotations

from typing import Optional

RTL_MODES = ("RTL", "SMART_RTL", "LAND", "AUTO_RTL", "QRTL", "QLAND")
STEPS = ("flying", "map_quiet", "rtl", "disarmed")


def touchdown(uav_ocs: Optional[dict], change_t: float, now: float, quiet_s: float, latch: dict) -> dict:
    """-> {flying, map_quiet, rtl, disarmed, go, step, text, quiet_for_s}; `latch` (the task's own dict) keeps flying / rtl once seen."""
    fl = (uav_ocs or {}).get("flight") or {}
    if fl.get("armed_seen") or fl.get("airborne_seen") or fl.get("armed") is True:
        latch["flying"] = True
    if str(fl.get("mode") or "").upper() in RTL_MODES and latch.get("flying"):
        latch["rtl"] = True
    flying = bool(latch.get("flying"))
    disarmed = flying and fl.get("armed") is False
    quiet_for = now - change_t if change_t > -1e8 else None
    map_quiet = quiet_for is None or quiet_for > float(quiet_s)          # never a map: nothing to wait for (the entry check says it)
    go = flying and disarmed and map_quiet
    st = {"flying": flying, "map_quiet": map_quiet, "rtl": bool(latch.get("rtl")), "disarmed": disarmed, "go": go,
          "quiet_for_s": None if quiet_for is None else round(quiet_for, 1)}
    if not flying:
        st["step"], st["text"] = "wait_flight", "waiting for the drone to take off"
    elif not disarmed:
        st["step"] = "rtl" if st["rtl"] else "flying"
        st["text"] = "drone returning (RTL / LAND)" if st["rtl"] else "drone flying - mapping"
    elif not map_quiet:
        st["step"], st["text"] = "map_quiet", f"drone down - map settling ({quiet_for:.0f} / {float(quiet_s):.0f} s)"
    else:
        st["step"], st["text"] = "touchdown", "TOUCHDOWN ✓ - drone disarmed, map quiet"
    return st
