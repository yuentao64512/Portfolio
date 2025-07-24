#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
waypoint.py

Plays back waypoints from a YAML file (mapping or list) and sequentially sends them to move_base. 
"""

import rospy
import yaml
import sys
import actionlib
import os
import rospkg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler


def get_default_waypoints_file():
    """Return default YAML path under limo_bringup/param/rtabmap."""
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('limo_bringup')
    return os.path.join(pkg_path, 'param', 'rtabmap', 'waypoints.yaml')


def load_waypoints(path):
    """Load waypoints from a YAML file. Supports both list and dict formats."""
    try:
        with open(path) as f:
            data = yaml.safe_load(f)
    except Exception as e:
        rospy.logerr("Failed to open waypoints file '%s': %s", path, e)
        sys.exit(1)

    raw = data.get('waypoints')
    if raw is None:
        rospy.logerr("YAML format error: 'waypoints' key not found in %s", path)
        sys.exit(1)

    if isinstance(raw, dict):
        # convert dict wp_0, wp_1... into list sorted by index
        try:
            items = sorted(raw.items(), key=lambda kv: int(kv[0].split('_')[1]))
            return [wp for _, wp in items]
        except Exception:
            return list(raw.values())

    if isinstance(raw, list):
        return raw

    rospy.logerr("YAML format error: 'waypoints' should be a list or dict, got %s", type(raw))
    sys.exit(1)


def make_goal(wp):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = wp['frame_id']
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = wp['x']
    goal.target_pose.pose.position.y = wp['y']
    q = quaternion_from_euler(0, 0, wp['yaw'])
    goal.target_pose.pose.orientation.x = q[0]
    goal.target_pose.pose.orientation.y = q[1]
    goal.target_pose.pose.orientation.z = q[2]
    goal.target_pose.pose.orientation.w = q[3]
    return goal


def main():
    rospy.init_node('waypoint_follower')

    # Determine waypoints file (param or default)
    default_file = get_default_waypoints_file()
    path = rospy.get_param('~waypoints_file', default_file)
    rospy.loginfo("waypoint_follower: loading from %s", path)

    waypoints = load_waypoints(path)
    if not waypoints:
        rospy.logwarn("No waypoints found in %s", path)
        return

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    if not client.wait_for_server(rospy.Duration(30.0)):
        rospy.logerr("move_base action server not available")
        sys.exit(1)

    for i, wp in enumerate(waypoints):
        rospy.loginfo("-> Sending waypoint %d: (%.2f, %.2f, yaw=%.2f)", i, wp['x'], wp['y'], wp['yaw'])
        client.send_goal(make_goal(wp))
        client.wait_for_result()
        state = client.get_state()
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Reached waypoint %d", i)
        else:
            rospy.logwarn("Failed waypoint %d (state %d)", i, state)

    rospy.loginfo("All waypoints processed.")

if __name__ == '__main__':
    main()

