#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import math

def move():
    rospy.init_node('square_move', anonymous=False)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    vel = Twist()
    speed = 0.2
    distance = 2.0
    turn_speed = math.radians(45)
    angle = math.radians(90)

    for _ in range(4):
        vel.linear.x = speed
        vel.angular.z = 0.0
        t0 = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - t0 < distance / speed:
            pub.publish(vel)
            rate.sleep()

        vel.linear.x = 0.0
        pub.publish(vel)
        rospy.sleep(1)

        vel.angular.z = turn_speed
        t0 = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - t0 < angle / turn_speed:
            pub.publish(vel)
            rate.sleep()

        vel.angular.z = 0.0
        pub.publish(vel)
        rospy.sleep(1)

    vel.linear.x = 0.0
    vel.angular.z = 0.0
    pub.publish(vel)

if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass
