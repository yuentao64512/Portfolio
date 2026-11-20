"""v0.1.54.1 - the EKF bit: 0x100 is EKF_PRED_POS_HORIZ_REL, ABS is 0x200. A relative-only predicted position must not
pass the AUTO guard (FcuState.ekf_pos_ok -> VehicleFsm's require_ekf_for_auto)."""
import types

from pymavlink import mavutil

from optiprimex.companion import link as L
from optiprimex.companion.config import Config


def test_constants_pinned_to_pymavlink():
    assert L.EKF_PRED_POS_HORIZ_REL == mavutil.mavlink.EKF_PRED_POS_HORIZ_REL == 0x100
    assert L.EKF_PRED_POS_HORIZ_ABS == mavutil.mavlink.EKF_PRED_POS_HORIZ_ABS == 0x200
    assert L.EKF_POS_HORIZ_ABS == mavutil.mavlink.EKF_POS_HORIZ_ABS == 0x10


def _ok(flags):
    m = L.MavLink(Config(), endpoint="udpin:127.0.0.1:0")
    m._apply("EKF_STATUS_REPORT", types.SimpleNamespace(flags=flags))
    return m._state.ekf_pos_ok


def test_relative_only_prediction_is_not_a_position():
    assert _ok(0x01 | 0x02 | 0x04 | 0x08 | 0x100) is False      # relative only: the old code said True
    assert _ok(0x100) is False
    assert _ok(0x10) is True                                    # POS_HORIZ_ABS
    assert _ok(0x200) is True                                   # PRED_POS_HORIZ_ABS
    assert _ok(0x01) is False


def test_fake_fcu_sends_the_real_bits():
    src = open(L.__file__.replace("companion/link.py", "sim/fake_fcu.py")).read()
    assert "0x10 | 0x100 | 0x200" in src and '"rel"' in src


def test_usv_ui_uses_the_same_bits():
    src = open(L.__file__.replace("optiprimex/companion/link.py", "usv_ui/usv_ui.py")).read()
    assert "(1 << 4)" in src and "(1 << 9)" in src
