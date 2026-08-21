# Drone RC Control and Vision Prototype

This drone project connects YOLO camera detections to RC/mission data. It maps an object's image position into a 100 x 100 grid and transmits JSON messages through a serial radio link. Separate scripts test radio latency and packet exchange.

## Detection and application cases

- Detect a target from the drone camera and report its grid position to the RC or mission controller.
- Feed marker positions into drone control or mission logic.
- Test a 433 MHz serial radio link before a field mission.
- Measure round-trip latency, timeouts, and basic packet reliability.

## Files

- `grid_sending.py`: YOLO camera inference, grid conversion, and serial transmission.
- `network.py`: repeated ping/latency test.
- `talker.py`: continuous JSON ping sender and response statistics.

## Configure

Edit the constants near the top of each script:

- `MODEL_PATH`: trained YOLO checkpoint.
- `RADIO_PORT` or `PORT`: Windows COM port.
- `BAUD`: radio baud rate; currently 57600.
- `CAMERA_INDEX`: attached camera number.
- `GRID_ROWS`, `GRID_COLS`, confidence, and send interval.

## Run

```powershell
python network.py
python grid_sending.py
```

Test with the drone disarmed and propellers removed first. This code reports perception and radio data; it does not itself provide flight stabilization, collision avoidance, route planning, or safety interlocks.
