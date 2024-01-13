#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import (Point, Pose, PoseWithCovariance, Quaternion,
                               Twist, TransformStamped,TwistWithCovariance, Vector3)
from nav_msgs.msg import Odometry
import tf_transformations
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Header
import math
import numpy as np
import time  # Importing the time library
from typing import NamedTuple


class PubOdomNode(Node):
    def __init__(self):
        super().__init__('PubOdomNode')
        queqe_size = 10
        # Publisher
        self.publisher = self.create_publisher(Odometry, 'odom', queqe_size)
        self.timer = self.create_timer(0.01, self.timer_callback)

        # Subscribers
        self.subscription = self.create_subscription(
            Twist,
            'feedback_odom',
            self.feedback_callback,
            queqe_size)
        
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        # Initialize the transform broadcaster
        self.tf_br = TransformBroadcaster(self)
        self.isOdomUpdate = False
        self.odom_output = Odometry(
            header=Header(
                stamp=self.get_clock().now().to_msg(),
                frame_id='odom'
            ),
            child_frame_id='base_footprint',
            pose=PoseWithCovariance(
                pose=Pose(
                    position=Point(
                        x=0.0,
                        y=0.0,
                        z=0.0
                    ),
                    orientation=Quaternion(
                        x=0.0,
                        y=0.0,
                        z=0.0,
                        w=1.0
                    )
                )
            ),
            twist=TwistWithCovariance(
                twist=Twist(
                    linear=Vector3(
                        x=0.0,
                        y=0.0,
                        z=0.0
                    ),
                    angular=Vector3(
                        z=0.0
                    )
                )
            )
        )

    def feedback_callback(self, msg):
        current_time = self.get_clock().now()
    
        if not hasattr(self, 'last_callback_time'):
            self.last_callback_time = current_time
            return

        dt = (current_time - self.last_callback_time).to_msg().nanosec * 1e-9
        self.last_callback_time = current_time

        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.wz = msg.angular.z
        
        delta_x = (self.vx * math.cos(self.th) - self.vy * math.sin(self.th)) * dt
        delta_y = (self.vx * math.sin(self.th) + self.vy * math.cos(self.th)) * dt
        delta_th = self.wz * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

    def timer_callback(self):
        quaternion = tf_transformations.quaternion_from_euler(0.0, 0.0, self.th)
        # Create Odometry message and fill in the data
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()  # Update time stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        odom_msg.pose.pose = Pose(
            position=Point(x=self.x, y=self.y, z=0.0),
            orientation=Quaternion(
            x=quaternion[0],
            y=quaternion[1],
            z=quaternion[2],
            w=quaternion[3]
        )
        )
        odom_msg.twist.twist.linear = Vector3(x=self.vx, y=self.vy, z=0.0)
        odom_msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=self.wz)

        # Publish the Odometry message
        self.publisher.publish(odom_msg)

         # Broadcast transform
        transform = TransformStamped()
        transform.header.stamp = odom_msg.header.stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'  # Make sure it matches the child frame ID in odom_output
        transform.transform.translation.x = odom_msg.pose.pose.position.x
        transform.transform.translation.y = odom_msg.pose.pose.position.y
        transform.transform.translation.z = odom_msg.pose.pose.position.z
        transform.transform.rotation = odom_msg.pose.pose.orientation

        self.tf_br.sendTransform(transform)
    
def main(args=None):
    rclpy.init(args=args)
    pub_odom_node = PubOdomNode()
    rclpy.spin(pub_odom_node)
    pub_odom_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()