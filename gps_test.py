import serial
import rospy
from time import sleep
from gpsmath import *

lonold = 0
latold = 0
#lon_init = ""
#lat_init = ""
lon_init = 103.8715875
lat_init = 1.327803167
#lon_init = 103.8710175
#lat_init = 1.327023167
# rospy.init_node("gps_node")
ser = serial.Serial()

def open_Serial():
	global ser

	if ser.isOpen():
		return 1

	ser.port = "/dev/ttyUSB0"
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
	global lonold, latold, lon_init, lat_init
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
			if lon_init == "" and lat_init == "":
				lon_init = longitude
				lat_init = latitude
			else:
				pass
			dist_from_init = haversine(lon_init, lat_init, longitude, latitude)
			dist_from_init = dist_from_init / 1000.0
			distInitString = "%10s %.10f m"%("Dist from init: ", dist_from_init)
			speed = haversine(longitude, latitude, lonold, latold)
			speed = speed / 1000.0
			speedString = "%10s %.10f m\n"%("Speed: ", speed)
			locationString = lonString + latString + speedString + distInitString
			latold = latitude
			lonold = longitude
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
	# gps_pub = rospy.Publisher()
	while open_Serial():
	# while True:
		try:
			dataList = readData()
			if type(dataList) is list:
				print printData(dataList)

		except KeyboardInterrupt:
			ser.close()
			break

	print "Is serial open? ", ser.isOpen()

