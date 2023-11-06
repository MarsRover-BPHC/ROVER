import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time
import datetime

sensor = 17

GPIO.setmode(GPIO.BCM)
GPIO.setup(sensor,GPIO.IN)

print ("IR Sensor Ready.....")

flag =0
first=0
second=0
omega=0.0
def omega_():
    try:
        while first==0:
            read=GPIO.input(sensor)
            if read:
                first=datetime.datetime.now()
        while True:
            read=GPIO.input(sensor)
            if read:
                second=datetime.datetime.now()
                difference=second-first
                omega=3.14/(2*difference.total_seconds())
                print("omega=",omega)
                first=second
                return String(omega)
                
    except KeyboardInterrupt:
        GPIO.cleanup()

def publish_message():
    # Initialize the ROS node
    rospy.init_node('message_publisher', anonymous=True)

    # Create a publisher with topic name 'chatter' and message type 'String'
    pub = rospy.Publisher('omega', String, queue_size=10)

    # Set the publishing rate (in Hz)
    rate = rospy.Rate(5)  # 1 Hz

    while not rospy.is_shutdown():
        # Create a message object
        message = String()
        message.data = omega_()

        # Publish the message
        pub.publish(message)

        # Sleep for the specified rate
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_message()
    except rospy.ROSInterruptException:
        pass


time.sleep(0.05)
