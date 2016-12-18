#!/usr/bin/env python

import rospy
import serial
import string
from std_msgs.msg import String

ser = serial.Serial()
ser.port = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Uno_75439333335351412220-if00"
#ser.port = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_95333353836351012281-if00"
#ser.port = "/dev/serial/by-path/pci-0000:00:1d.0-usb-0:1.3:1.0"  #depends on the device port name
ser.baudrate = 9600
ser.open()

def compass():
	pub = rospy.Publisher('compass', String, queue_size = 100)
	rospy.init_node('compass', anonymous=True)
	rate = rospy.Rate(100)
	#rospy.loginfo(str("test"))
	while ser.isOpen():
		#rospy.loginfo(str("Reading data from serial port"))
		bytesToRead = ser.readline()
		#rospy.loginfo(bytesToRead)
		bytesToRead = bytesToRead.strip('\n')
		if len(bytesToRead)  > 0: 
			#removes all alphabets turns into integer to remove 00
			all = string.maketrans('','')
			nodigs = all.translate(all, string.digits)
			bytesToRead = bytesToRead.translate(all, nodigs)
			bytesToPublish = int(bytesToRead)
			#publishing data in string for standardization
			#rospy.loginfo(str(bytesToRead))
			pub.publish(str(bytesToPublish))

		rate.sleep()

if __name__ == '__main__':
	try:
		compass()
	except rospy.ROSInterruptException:
		#ser.close()  #this doesn't seem to work well
		pass
		
		
