"""Layer 2: task behaviours (sub-state machines inside V4)."""
from .base import Task, TaskContext, TaskStatus  # noqa: F401
from .t4_incident import T4Incident  # noqa: F401
from .goto_task import GotoTask  # noqa: F401
from .test_tasks import TestManual, TestAuto  # noqa: F401
from .t1_safe_passage import T1SafePassage  # noqa: F401
from .test_path import TestPath  # noqa: F401
from .t3_logistics import T3Logistics  # noqa: F401
from .test_sway import TestSway  # noqa: F401
from .box_inspection_task import BoxInspectionTask  # noqa: F401
from .return_home import ReturnHome  # noqa: F401
from .test_orbit import TestOrbit  # noqa: F401
from .scout import Scout  # noqa: F401
from .test_detect import TestDetect  # noqa: F401
from .test_guard import TestGuard  # noqa: F401
from .test_yaw import TestYaw  # noqa: F401

REGISTRY = {"T4": T4Incident, "GOTO": GotoTask, "TEST_MANUAL": TestManual, "TEST_AUTO": TestAuto, "T1": T1SafePassage,
            "TEST_PATH": TestPath, "T3": T3Logistics, "TEST_SWAY": TestSway, "BOX_INSPECT": BoxInspectionTask,
            "RETURN_HOME": ReturnHome, "TEST_ORBIT": TestOrbit,           # v0.1.30 (WP8c)
            "SCOUT": Scout, "TEST_DETECT": TestDetect,                     # v0.1.79 (WP35)
            "TEST_GUARD": TestGuard, "TEST_YAW": TestYaw}  # v0.1.85 (WP41 T0.5)                                        # v0.1.80 (WP36)
