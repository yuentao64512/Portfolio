# Navigation beacons

Gazebo Harmonic / gz-sim8 system plugin. Both USV world files load the built
library in `usv_sim_workspace/lib`. The combined boat/drone launcher rebuilds
it automatically. For other launchers, build it once after copying the workspace:

```bash
cmake -S plugins/navigation_beacons -B plugins/navigation_beacons/build
cmake --build plugins/navigation_beacons/build -j 2
ctest --test-dir plugins/navigation_beacons/build --output-on-failure
```

Timing uses simulation time and a 50% duty cycle:

- Entrance blue: 0.5 Hz, 1 second on / 1 second off.
- Exit blue: steady. Set `exit_hz` to `1.0` in both worlds if desired.
- Other buoy lights and the six dock beacons: 1 Hz, 0.5 second on / off.
- Sun and decorative waterfront lighting are not navigation beacons and remain steady.

The system changes lamp material emission and bulb brightness as well as point
light intensity. Buoy body paint and all on-state colors are preserved. Black
hazards retain their original dark color and dim lights; they do not become white.
Runtime-spawned docks are discovered automatically. Frequencies follow simulation
time, so a slow simulation also displays slower blinking in real time.

The controlled boat spawns at world `(0, 37, 0.12)` with yaw `-pi/2`: outside the
green dock's mouth at `y=41`, facing south toward the course. The decorative
`robotx_usv_boat_start` model is not the controlled boat.

Tests cover blink boundaries, original color/intensity preservation, steady exit,
unaffected sun, pause/reset, and dock lights inserted after startup.
