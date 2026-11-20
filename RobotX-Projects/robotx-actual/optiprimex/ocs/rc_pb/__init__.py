"""RoboNation's RobotX 2026 protobuf classes (vendored, see PROVENANCE.md). Import from here:

    from optiprimex.ocs.rc_pb import common_pb2, rx_reports_pb2, rx_requests_pb2, rx_commands_pb2, rx_course_pb2, rx_common_pb2
"""
import os as _os
import sys as _sys

_HERE = _os.path.dirname(_os.path.abspath(__file__))
if _HERE not in _sys.path:
    _sys.path.insert(0, _HERE)          # the generated code imports `common_pb2` and `robotx.*` top-level

import common_pb2  # noqa: E402,F401
from robotx import rx_common_pb2, rx_commands_pb2, rx_course_pb2, rx_reports_pb2, rx_requests_pb2  # noqa: E402,F401
