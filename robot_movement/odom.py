import rospy
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from std_msgs.msg import Float64
from tf.transformations import quaternion_from_euler
from math import cos, sin

class WheelEncoderOdom:
    def __init__(self):
        rospy.init_node('wheel_encoder_odom_publisher')

        # Left wheel angular velocity
        self.left_wheel_angular_velocity_sub = rospy.Subscriber('left_omega', Float64, self.left_wheel_angular_velocity_callback)
        self.right_wheel_angular_velocity_sub = rospy.Subscriber('right_omega', Float64, self.right_wheel_angular_velocity_callback)

        # Odometry publisher
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

        # Robot parameters
        self.wheel_separation = 0.2  # Replace with your robot's wheel separation distance
        self.wheel_radius = 0.05  # Replace with your robot's wheel radius

        # Initialize odometry variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.right_angular_velocity = 0.0
        self.left_angular_velocity = 0.0

    def left_wheel_angular_velocity_callback(self, data):
        self.left_angular_velocity = data.data
        self.update_odometry()

    def right_wheel_angular_velocity_callback(self, data):
        self.right_angular_velocity = data.data
        self.update_odometry()

    def update_odometry(self, right_angular_velocity=0.0):
        # Use self.left_angular_velocity and self.right_angular_velocity to update odometry

        # Calculate linear and angular velocity
        linear_velocity = (self.wheel_radius / 2.0) * (self.left_angular_velocity + right_angular_velocity)
        angular_velocity = (self.wheel_radius / self.wheel_separation) * (self.right_angular_velocity - self.left_angular_velocity)

        print("linear vel:", linear_velocity, "/n", "angular velocity:", angular_velocity, "/n")
        self.theta += angular_velocity * rospy.get_time()
        self.x += linear_velocity * rospy.get_time() * cos(self.theta)
        self.y += linear_velocity * rospy.get_time() * sin(self.theta)

        # Create and publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Set the position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Convert the yaw angle to quaternion
        quaternion = quaternion_from_euler(0.0, 0.0, self.theta)
        odom_msg.pose.pose.orientation = Quaternion(*quaternion)

        # Set the linear and angular velocities
        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.angular.z = angular_velocity

        # Publish odometry message
        self.odom_pub.publish(odom_msg)

if __name__ == '__main__':
    try:
        wheel_encoder_odom = WheelEncoderOdom()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
