"""Marina Bay defaults (v0.1.33, ROADMAP 7a / WP10b): RoboNation's own Course Alpha, as rc_test/test_server.py publishes it.

A true rectangle, 30.1 x 25.0 m, corners 90.0 deg, the long side on bearing 032 / 212 deg, its centre 55 m west of the Marina Bay City
Gallery shore point. The simulator's worlds, the bench profiles, the fake FCU's start and the fake judge's course use it, so the
simulator, the profiles and rc-test agree to the metre; the real course replaces it when RoboCommand publishes one.
"""
CENTRE = (1.2810202, 103.8550097)
COURSE_ALPHA = [(1.2809652, 103.8548424), (1.2808457, 103.8550331), (1.2810752, 103.8551770), (1.2811947, 103.8549863)]   # open ring
TRANSIT_A = [(1.281294119, 103.855048646), (1.281174598, 103.855239379), (1.2810752, 103.8551770), (1.2811947, 103.8549863)]
