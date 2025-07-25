#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from pcl_ros import point_cloud2
from geometry_msgs.msg import Twist

def cloud_cb(msg):
    # convert to Nx3 numpy array
    points = np.array(list(point_cloud2.read_points(msg, skip_nans=True)))
    # RANSAC plane fit (you could use sklearn or Open3D here)
    # ... fit plane, get normal vector ...
    normal = fit_plane_normal(points)         # your RANSAC helper
    slope_angle = np.degrees(np.arccos(abs(normal.dot([0,0,1]))))
    
    # split inliers/outliers
    ground_mask = np.abs(np.dot(points, normal) + d) < ground_dist_thresh
    cloud_no_ground = points[~ground_mask]
    # republish cloud_no_ground as PointCloud2 on /cloud_no_ground …
    
    # if slope too steep, slow down
    twist = Twist()
    if slope_angle > MAX_SLOPE_DEG:
        twist.linear.x = SLOW_SPEED
    else:
        twist.linear.x = NORMAL_SPEED
    cmd_pub.publish(twist)

if __name__ == "__main__":
    rospy.init_node("slope_detector")
    ground_dist_thresh = rospy.get_param("~ground_distance", 0.05)
    MAX_SLOPE_DEG    = rospy.get_param("~max_slope_deg", 15)
    SLOW_SPEED      = rospy.get_param("~slow_speed", 0.1)
    NORMAL_SPEED    = rospy.get_param("~normal_speed", 0.5)

    cloud_sub = rospy.Subscriber("/camera/depth/points", PointCloud2, cloud_cb)
    cmd_pub   = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    # (Also create a publisher for the filtered PointCloud2)
    rospy.spin()

