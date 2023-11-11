#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64
import time
import serial

# Serial port configuration
SERIAL_PORT = '/dev/ttyACM0'  # Replace with the correct port for your Arduino
BAUD_RATE = 9600

# ROS node initialization
rospy.init_node('motor_velocity_publisher')

# Serial port initialization
serial_port = serial.Serial(SERIAL_PORT, BAUD_RATE)

# Variables to store left and right velocities
left_velocity = 0.0
right_velocity = 0.0

# ROS message callback function for left motor
def left_velocity_callback(data):
    global left_velocity
    left_velocity = data.data

# ROS message callback function for right motor
def right_velocity_callback(data):
    global right_velocity
    right_velocity = data.data

# ROS publisher initialization
pub = rospy.Publisher('velocity_string', String, queue_size=10)

# ROS subscriber initialization
rospy.Subscriber('left_motor_velocity', Float64, left_velocity_callback)
rospy.Subscriber('right_motor_velocity', Float64, right_velocity_callback)

# ROS spin
rate = rospy.Rate(10)  # Adjust the rate as per your requirement
while not rospy.is_shutdown():
    if left_velocity > 0:
        lv= 1
    elif left_velocity < 0:
        lv = 2
    else:
        lv = 0
    if right_velocity > 0:
        rv= 1
    elif right_velocity < 0:
        rv = 2
    else:
        rv = 0
    velocity_string = '#@'+str(lv) + ',' + str(rv)+'@#'

    # Publish as ROS message
    rospy.loginfo(velocity_string)
    pub.publish(velocity_string)

    # Send as serial output
    serial_port.write(velocity_string.encode())

    rate.sleep()
    time.sleep(1)
# Close serial port on shutdown
serial_port.close()