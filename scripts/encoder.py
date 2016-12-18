#!/usr/bin/env python
import rospy
import serial
import string
from std_msgs.msg import String

ser = serial.Serial()
ser.port = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Uno_75439333335351412220-if00"
#ser.port = "/dev/ttyACM1"  #depends on the device port name
ser.baudrate = 9600
ser.open()

def encoder():
	pub = rospy.Publisher('encoder', String, queue_size = 10)
	rospy.init_node('encoder', anonymous=True)
	rate = rospy.Rate(20)
	while ser.isOpen():
                        bytesToRead = ser.readline()
                        #rospy.loginfo(str(bytesToRead))
                        bytesToRead = bytesToRead.strip('\n')
                        if len(bytesToRead)  == 17: 
                                #separates the data into readable things
                                #rospy.loginfo(str(bytesToRead))
                                r_encoder, r_direction, l_encoder, l_direction = bytesToRead.split(" ")
                                nr_encoder = int(r_encoder);
                                nl_encoder = int (l_encoder);
                                if r_direction == "1" :
                                        nr_encoder = -int(nr_encoder)
                                elif l_direction == "1" :
                                        nl_encoder = -int(nl_encoder)
                                else :
                                        nr_encoder = int(nr_encoder)
                                        nl_encoder = int(nl_encoder)

		#turning them into strings
                                bytesToPublish = '%d %d' % (nl_encoder, nr_encoder)

                                #publishing data in string for standardization
                                #rospy.loginfo(str(bytesToPublish))
                                pub.publish(str(bytesToPublish))
                        rate.sleep()

if __name__ == '__main__':
	try:
		encoder()
	except rospy.ROSInterruptException:
		#ser.close()  #this doesn't seem to work well
		pass
		
		
