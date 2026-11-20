# Guided-mode Pixhawk to Gazebo workflow

Terminal 1:

```bash
bash run_mavlink_gazebo.sh
```

Terminal 2:

```bash
bash run_mavlink_gazebo_guided_move.sh --duration 5 --magnitude 0.20
```

The second script requests `GUIDED`, verifies the heartbeat reports `GUIDED`,
then sends the crab commands through the FCU output bridge. It uses SERVO2 for
the front thruster, SERVO4 for the rear thruster, and rear trim `1.4`.

This is Guided mode with direct lateral servo commands; ArduPilot Rover does
not provide a native Guided crab/sway axis through `MANUAL_CONTROL`.
