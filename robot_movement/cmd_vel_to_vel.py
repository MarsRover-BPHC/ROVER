import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

# Constants
WHEEL_DISTANCE = 0.5  # Distance between the two wheels in meters

# Variables
left_velocity = 0.0
right_velocity = 0.0

def cmd_vel_callback(msg):
    global left_velocity, right_velocity

    # Calculate linear and angular velocities
    linear_velocity = msg.linear.x
    angular_velocity = msg.angular.z

    # Calculate left and right wheel velocities
    left_velocity = linear_velocity - (angular_velocity * WHEEL_DISTANCE / 2.0)
    right_velocity = linear_velocity + (angular_velocity * WHEEL_DISTANCE / 2.0)

def publish_motor_velocities():
    global left_velocity, right_velocity

    # Create publishers for left and right motor velocities
    left_motor_pub = rospy.Publisher('left_motor_velocity', Float64, queue_size=10)
    right_motor_pub = rospy.Publisher('right_motor_velocity', Float64, queue_size=10)

    rospy.init_node('motor_control')

    rate = rospy.Rate(10)  # Publishing rate of 10 Hz

    while not rospy.is_shutdown():
        # Publish left and right motor velocities
        left_motor_pub.publish(Float64(left_velocity))
        right_motor_pub.publish(Float64(right_velocity))

        rate.sleep()

if __name__ == '__main__':
    try:
        # Subscribe to cmd_vel topic
        rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback)

        # Call the function to publish motor velocities
        publish_motor_velocities()
    except rospy.ROSInterruptException:
        pass