import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import math
# Constants
WHEEL_DISTANCE = 0.5  # Distance between the two wheels in meters
WHEEL_RADIUS = 0.1  # Radius of the wheels in meters

# Variables
left_velocity = 0.0
right_velocity = 0.0
odom_publisher = None
tf_broadcaster = None

def left_velocity_callback(msg):
    global left_velocity
    left_velocity = msg.data

def right_velocity_callback(msg):
    global right_velocity
    right_velocity = msg.data

def publish_odom():
    global left_velocity, right_velocity

    rate = rospy.Rate(10)  # Publishing rate of 10 Hz

    # Variables for odometry calculation
    x = 0.0
    y = 0.0
    theta = 0.0
    previous_time = rospy.Time.now()

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        dt = (current_time - previous_time).to_sec()

        # Calculate linear and angular velocities
        linear_velocity = (left_velocity + right_velocity) * WHEEL_RADIUS/ 2.0
        angular_velocity = (right_velocity - left_velocity) * WHEEL_RADIUS / WHEEL_DISTANCE

        # Calculate change in position and orientation
        delta_x = linear_velocity * dt * math.cos(theta)
        delta_y = linear_velocity * dt * math.sin(theta)
        delta_theta = angular_velocity * dt

        # Update position and orientation
        x += delta_x
        y += delta_y
        theta += delta_theta

        # Publish Odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = math.sin(theta/2.0)
        odom.pose.pose.orientation.w = math.cos(theta/2.0)

        # Publish TF transform
        tf_broadcaster.sendTransform(
            (x, y, 0),
            (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w),
            current_time,
            "base_link",
            "odom"
        )

        # Publish Odometry message
        odom_publisher.publish(odom)

        previous_time = current_time
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('odometry_publisher')

    # Create publishers for odom and tf
    odom_publisher = rospy.Publisher('odom', Odometry, queue_size=10)
    tf_broadcaster = tf.TransformBroadcaster()

    # Subscribe to left and right motor velocities
    rospy.Subscriber('left_motor_velocity', Float64, left_velocity_callback)
    rospy.Subscriber('right_motor_velocity', Float64, right_velocity_callback)

    publish_odom()

    rospy.spin()