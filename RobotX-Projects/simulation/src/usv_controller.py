#!/usr/bin/env python3

import time

from gz.msgs10.double_pb2 import Double
from gz.msgs10.odometry_pb2 import Odometry
from gz.transport13 import Node


class USVController:
    def __init__(self):
        self.node = Node()

        self.model_name = "blueboat"
        self.left_thrust_topic = f"/model/{self.model_name}/joint/motor_port_joint/cmd_thrust"
        self.right_thrust_topic = f"/model/{self.model_name}/joint/motor_stbd_joint/cmd_thrust"
        self.odom_topic = f"/model/{self.model_name}/odometry"

        self.left_pub = self.node.advertise(self.left_thrust_topic, Double)
        self.right_pub = self.node.advertise(self.right_thrust_topic, Double)
        self.node.subscribe(self.odom_topic, self.odom_callback)

        self.current_x = 0.0
        self.current_y = 0.0
        self.running = True

        print("USV Controller Started!")
        print(f"Publishing to: {self.left_thrust_topic}")
        print(f"Publishing to: {self.right_thrust_topic}")
        print(f"Subscribing to: {self.odom_topic}")
        print("=" * 50)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        print(f"Boat Position: ({self.current_x:.2f}, {self.current_y:.2f})")

    def send_command(self, port_thrust=0.0, stbd_thrust=0.0):
        left_msg = Double()
        left_msg.data = port_thrust
        right_msg = Double()
        right_msg.data = stbd_thrust
        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)

    def run(self):
        print("Sending test command: port=12.0, stbd=8.0")

        try:
            while self.running:
                self.send_command(port_thrust=12.0, stbd_thrust=8.0)
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nShutting down controller...")
            self.running = False
            self.send_command(port_thrust=0.0, stbd_thrust=0.0)


if __name__ == "__main__":
    controller = USVController()
    controller.run()
