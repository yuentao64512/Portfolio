DECLARATION = "/robocommand/team/declaration"
RUN_START = "/robocommand/run/start"
TEAM_GO = "/robocommand/team/go"
CONTROL_STOP = "/robocommand/control/stop"
COURSE = "/robocommand/course"
UAV_REPORT = "/uav/perception/buoys"
USV_UAV_REPORT = "/local/usv/uav/perception/buoys"
ROUTE_CHANGE = "/robocommand/route/change"
STATUS_UAV = "/robocommand/status/uav1"
STATUS_USV = "/robocommand/status/usv1"
USV_CV = "/usv/perception/sim_camera"
USV_ROUTE = "/usv/mission/sim_route"
USV_DRIVE_STATUS = "/usv/mission/sim_drive_status"


def heartbeat(vehicle_id: str) -> str:
    return f"/robocommand/heartbeat/{vehicle_id.lower()}"

