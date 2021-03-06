#!/usr/bin/env python

import serial
import rospy
from time import sleep
from gps_common.msg import GPSFix

ser = serial.Serial()
f = GPSFix()

def open_Serial():
	global ser

	if ser.isOpen():
		return 1

	ser.port = "/dev/ttyACM0"
	ser.baudrate = 9600
	try:
		ser.open()
	except serial.serialutil.SerialException as ex:
		return 0
	if ser.isOpen():
		return 1
	return 0

def time_Conversion(timeStr):
	if len(timeStr) == 9:
		hour = timeStr[0:2]
		hour = int(hour) + 8
		while hour > 24:
			hour = hour - 24
		hour = str(hour)
		minute = timeStr[2:4]
		second = timeStr[4:]
		timeString = "%10s %s:%s:%s UTC+8"%("Time:",hour,minute,second)
		return timeString
	else:
		return "%10s time error"%"Time:"

def latlon_Conversion(latlonArray):
	global f
	if len(latlonArray) == 4:
		if len(latlonArray[0]) == 10 and len(latlonArray[2]) == 11:
			lat = latlonArray[0]
			latdeg = int(lat[0:2])
			latmin = float(lat[2:])
			latitude = latdeg + latmin / 60
			latPole = latlonArray[1]
			if latPole == 'S':
				latitude = latitude * -1
			lon = latlonArray[2]
			londeg = int(lon[0:3])
			lonmin = float(lon[3:])
			longitude = londeg + lonmin / 60
			lonPole = latlonArray[3]
			if lonPole == 'W':
				longitude = longitude * -1
			latString = "%10s %.10f\n"%("Latitude:",latitude)
			lonString = "%10s %.10f\n"%("Longitude:",longitude)
			f.longitude = longitude
			f.latitude = latitude
			locationString = lonString + latString
			return locationString
		else:
			return "%10s lat lon error"%"Location:"
	else:
		return "%10s lat lon error"%"Location:"

def fix(fixQuality):
	if len(fixQuality) == 1:
		try:
			fixQuality = int(fixQuality)
			if fixQuality == 0:
				fixString = "Invalid"
			elif fixQuality == 1:
				fixString = "GPS fix"
			elif fixQuality == 2:
				fixString = "DGPS fix"
			elif fixQuality == 3:
				fixString = "PPS fix"
			else:
				return "%10s fix error"%"Fix:"
			f.status.status = fixQuality - 1
			return "%10s %d %s"%("Fix:",fixQuality,fixString)
		except ValueError:
			return "%10s fix error"%"Fix:"
	else:
		return "%10s fix error"%"Fix:"

def satellite(satno):
	if len(satno) == 2:
		satno = int(satno)
		return "%10s %d"%("Satellite:",satno)
	else:
		return "%10s satellite error"%"Satellite:"

def readData():
	global ser
	ser.flush()
	bytesToRead = ser.readline()
	# bytesToRead = "$GPGGA2"
	# bytesToRead = "$GPGGA,192233.44,0119.66279,S,10352.29525,E,0,07,8,9,10,11,12,13,14*15\r\n"
	try:
		if "$GPGGA" in bytesToRead:
			print bytesToRead
			bytesToRead = bytesToRead.strip('\r\n')
			bytesToRead = bytesToRead.split(",")
			bytesToRead.append(bytesToRead[-1])
			bytesToRead[-2] = bytesToRead[-2].split("*")[0]
			bytesToRead[-1] = bytesToRead[-1].split("*")[1]
			return bytesToRead
	except IndexError:
			return "data error"

def printData(List):
	try:
		return "%s\n%s\n%s\n%s\r\n"%(time_Conversion(List[1]),latlon_Conversion(List[2:6]),fix(List[6]),satellite(List[7]))
	except TypeError:
		return 

if __name__ == '__main__':
	rospy.init_node('gps_client')
	gps_pub = rospy.Publisher('extended_fix', GPSFix, queue_size = 10)
	try:
		while open_Serial() and not rospy.is_shutdown():
		
			dataList = readData()
			if type(dataList) is list:
				gps_pub.publish(f)
				print printData(dataList)

	except rospy.ROSInterruptException or KeyboardInterrupt:
		ser.close()
		pass

	print "Is serial open? ", ser.isOpen()

