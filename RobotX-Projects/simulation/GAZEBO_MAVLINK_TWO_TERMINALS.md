# Pixhawk to Gazebo: two-terminal workflow

Use two terminals from the repository root.

Terminal 1 starts only the four-thruster Gazebo GUI and stays open:

```bash
./run_mavlink_gazebo.sh
```

Terminal 2 sends the movement command through the Pixhawk. It does not start
or stop Gazebo:

```bash
./run_mavlink_gazebo_move.sh --duration 5 --magnitude 0.20
```

The default mapping is `SERVO2` to the bow lateral thruster and `SERVO4` to the
stern lateral thruster. The default rear trim is `1.4`; the default PWM plan is
front `1400`, rear `1360`. The port and starboard Gazebo thrusters remain zero.

Use the movement script with `--dry-run` to calculate the command without
sending it:

```bash
./run_mavlink_gazebo_move.sh --dry-run --duration 5 --magnitude 0.20
```

The bridge writes `mavlink_gazebo_crab.jsonl`. Convert its state transitions to
Mermaid with:

```bash
./scripts/fcu_state_to_mermaid.py mavlink_gazebo_crab.jsonl \
  --output mavlink_gazebo_crab.mmd
```
