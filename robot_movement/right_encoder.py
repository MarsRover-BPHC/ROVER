import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
import RPi.GPIO as GPIO
import time
import datetime

sensor = 23

GPIO.setmode(GPIO.BCM)
GPIO.setup(sensor, GPIO.IN)

print("IR Sensor Ready.....")

flag = 0
first = 0
second = 0
omega = 0.0


def omega_():
    try:
        global first, second, omega
        while first == 0:
            read = GPIO.input(sensor)
            if read:
                first = datetime.datetime.now()
        while True:
            read = GPIO.input(sensor)
            if read:
                second = datetime.datetime.now()
                difference = second - first
                omega = 3.14 / (2 * difference.total_seconds())
                print("Left omega=", omega)
                first = second
                return omega

    except KeyboardInterrupt:
        GPIO.cleanup()


def publish_message():
    # Initialize the ROS node
    rospy.init_node('message_publisher', anonymous=True)

    # Create a publisher with topic name 'left_omega' and message type 'Float64'
    pub = rospy.Publisher('right_omega', Float64, queue_size=10)

    # Set the publishing rate (in Hz)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # Get the angular velocity
        angular_velocity = omega_()

        # Publish the angular velocity
        pub.publish(angular_velocity)

        # Sleep for the specified rate
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_message()
    except rospy.ROSInterruptException:
        pass

time.sleep(0.05)
