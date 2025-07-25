#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
record_waypoints.py

Interactively record the robot’s current map-frame pose into a YAML waypoints file,
overwriting any existing file on each run.
Press ENTER to capture a new waypoint, or type 'q' to quit.
Default YAML path is <limo_bringup>/param/rtabmap/waypoints.yaml.
"""

import rospy
import yaml
import os
import tf2_ros
import rospkg
from tf.transformations import euler_from_quaternion


def get_default_filepath():
    """Return default YAML path under limo_bringup/param/rtabmap."""
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('limo_bringup')
    return os.path.join(pkg_path, 'param', 'rtabmap', 'waypoints.yaml')


def get_current_pose(tf_buffer):
    """Lookup the current transform from 'map' to 'base_link' and return (x, y, yaw)."""
    trans = tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(1.0))
    x = trans.transform.translation.x
    y = trans.transform.translation.y
    q = trans.transform.rotation
    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    return x, y, yaw


def main():
    rospy.init_node('record_waypoints', anonymous=True)

    # Parameter: path to YAML file (default under package param/rtabmap)
    filename = rospy.get_param('~file', get_default_filepath())
    rospy.loginfo("record_waypoints: saving to '%s' (overwriting)", filename)

    # Ensure output directory exists
    directory = os.path.dirname(filename)
    if not os.path.isdir(directory):
        os.makedirs(directory)

    # Prepare TF listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Start fresh: do NOT load any existing waypoints
    waypoints = {}
    idx = 0

    print("\nPress ENTER to record current pose, or 'q'+ENTER to quit.")
    while not rospy.is_shutdown():
        choice = raw_input("[{}] > ".format(idx)).strip().lower()
        if choice == 'q':
            break
        try:
            x, y, yaw = get_current_pose(tf_buffer)
        except Exception as e:
            print("Failed to get pose: {}".format(e))
            continue

        name = "wp_{}".format(idx)
        waypoints[name] = {'frame_id': 'map', 'x': x, 'y': y, 'yaw': yaw}
        print("Recorded {}: x={:.3f}, y={:.3f}, yaw={:.3f}".format(name, x, y, yaw))
        idx += 1

        # Overwrite YAML on each capture
        with open(filename, 'w') as f:
            yaml.dump({'waypoints': waypoints}, f, default_flow_style=False)

    print("Finished. Saved {} waypoints to {}".format(len(waypoints), filename))

if __name__ == '__main__':
    main()

