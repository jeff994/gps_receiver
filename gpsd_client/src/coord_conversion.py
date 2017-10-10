#!/usr/bin/env python

import rospy
import coordTransform_utils as T
from sensor_msgs.msg import NavSatFix

gcj02 = False
GPS = False
longitude = 0.0
latitude = 0.0

def callback(data):
	global gcj02, GPS, longitude, latitude
	GPS = True
	longitude = data.longitude
	latitude = data.latitude
	if gcj02:
		longitude, latitude = T.wgs84_to_gcj02(longitude, latitude)

if __name__ == '__main__':
	rospy.init_node('coord_conversion')
	rospy.Subscriber('conv_fix', NavSatFix, callback)
	fix_pub = rospy.Publisher('fix', NavSatFix, queue_size = 10)

	try:
		while not rospy.is_shutdown():
			if GPS:
				f = NavSatFix()
				f.longitude = longitude
				f.latitude = latitude
				fix_pub.publish(f)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
