#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf.broadcaster import TransformBroadcaster
import math
class OdomPublisherNode:
    def __init__(self):
        rospy.init_node('odom_publisher_node')

        # Initialize variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Set the update rate (in Hz)
        self.rate = 10  # Update rate in Hz

        # Define the ROS subscribers
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

        # Define the ROS publishers
        self.odom_publisher = rospy.Publisher('/odom', Odometry, queue_size=10)

        # Create a TransformBroadcaster for publishing the tf message
        self.tf_broadcaster = TransformBroadcaster()

    def cmd_vel_callback(self, msg):
        # Update the x, y, and theta based on the received Twist message
        dt = 1.0 / self.rate  # Time difference between updates
        self.x += msg.linear.x * dt * math.cos(self.theta)
        self.y += msg.linear.y * dt * math.sin(self.theta)
        self.theta += msg.angular.z * dt

    def publish_odom_tf(self):
        # Publish the odometry message
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Convert theta to quaternion
        quaternion = quaternion_from_euler(0.0, 0.0, self.theta)
        odom.pose.pose.orientation.x = quaternion[0]
        odom.pose.pose.orientation.y = quaternion[1]
        odom.pose.pose.orientation.z = quaternion[2]
        odom.pose.pose.orientation.w = quaternion[3]

        # Set covariance matrix values
        odom.pose.covariance[0] = 0.1  # x position variance
        odom.pose.covariance[7] = 0.1  # y position variance
        odom.pose.covariance[35] = 0.2  # theta orientation variance

        self.odom_publisher.publish(odom)

        # Publish the tf message
        self.tf_broadcaster.sendTransform(
            (self.x, self.y, 0.0),
            quaternion,
            rospy.Time.now(),
            'base_link',
            'odom'
        )

    def run(self):
        while not rospy.is_shutdown():
            self.publish_odom_tf()
            rospy.sleep(1.0 / self.rate)

if __name__ == '__main__':
    try:
        node = OdomPublisherNode()
        node.run()
    except rospy.ROSInterruptException:
        pass