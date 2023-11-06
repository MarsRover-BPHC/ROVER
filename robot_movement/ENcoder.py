import RPi.GPIO as GPIO
import time
import datetime
import rospy
from std_msgs.msg import String

sensor = 17

GPIO.setmode(GPIO.BCM)
GPIO.setup(sensor,GPIO.IN)

print ("IR Sensor Ready.....")
flag =0
first=0
second=0
omega=0.0
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
        
except KeyboardInterrupt:
    GPIO.cleanup()
time.sleep(0.2)

