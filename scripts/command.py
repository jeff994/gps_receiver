#!/usr/bin/env python
import rospy
import serial
import string

from std_msgs.msg import String
from math import radians, cos, sin, asin, sqrt, atan2, degrees

###################### EDIT HERE ###########################
#defining or acquiring the GPS coordinates
gps_num = 4
start = [103.962386,1.340549]
gps_lon = [103.962386,103.962389,103.962456,103.962461,103.962381] #S,A,B,C,D
gps_lat = [1.340549,1.3407,1.340696,1.340589,1.340599]

initial_bearing = 0 	#set as north for now
loops = 1 			#how many rounds to go
encode_to_mm = 1000 #1000 encoding signals = 1 mm travelled

############################################################

compass_data = 0	#degrees, true north is 0 degrees
dist_travelled = 0	#mm
x_now = 0  	#mm
y_now = 0	#mm
r = 350 		#mm, distance between center of robot to wheel
x_target = 0	#mm
y_target = 0 	#mm, should always be 0, because we will be moving in a straight line
bearing_target = 0 	#degrees
job_des = []
job_num = []

#defining serial port to write to (the commands)
ser = serial.Serial()
ser.port = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_95333353836351012281-if00" #depends on the device port name
ser.baudrate = 9600
ser.open()

def haversine(lon1, lat1, lon2, lat2):
	#convert to radians
	lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1,lon2, lat2])
	#haversine
	dlon = lon2 - lon1
	dlat = lat2 - lat1
	a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
	c = 2 * asin(sqrt(a))
	r = 6371 #radius of earth in kilometers

	return c * r

def bearing(lon1, lat1, lon2, lat2):  #from position 1 to 2
	#convert to radians
	lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])
	
	bearing = atan2(sin(lon2-lon1)*cos(lat2), cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(lon2-lon1))
	bearing = degrees(bearing)
	bearing = (bearing + 360) % 360

	return bearing

def job_details(first_point, second_point):
	global gps_lon
	global gps_lat
	#handles from first_point to second_point
	distance = haversine(gps_lon[first_point],gps_lat[first_point],gps_lon[second_point],gps_lat[second_point]) #km
	angle_next = bearing(gps_lon[first_point],gps_lat[first_point],gps_lon[second_point],gps_lat[second_point]) #deg  
		
	#handles turning angle		
	#turn_angle = angle_next - angle_now
	#if turn_angle > 180.0 :
	#	turn_angle = turn_angle - 360.0
		
	#handles forward distance in mm
	distance = distance * 1000.0 * 1000.0

	return ([round(angle_next), distance])

def job_generator_straight_1m():
	global job_des
	global job_num
	job_num.extend([0, 1000]) 
	job_des.extend(['T','F'])


def job_generator(init_bearing, loops):
	global job_des
	global job_num
	
	#handles from start to first point
	job = job_details(0, 1)
	job_des.append('T');
	job_des.append('F');
	job_num.extend([job[0],job[1]])   #in the form of target bearing and distance
		
	#handles how many loops
	for i in range (loops) :
		for k in range (gps_num):
			if k < gps_num - 1 :
				job = job_details(k + 1, k + 2)
				job_des.extend(['T','F'])
				job_num.extend([job[0],job[1]])
				
			else : 
				job = job_details(k + 1, 1)
				job_des.extend(['T','F'])
				job_num.extend([job[0],job[1]])
	
	#handles closing loop, going back to start
	job = job_details(1, 0)
	job_des.extend(['T','F'])
	job_num.extend([job[0],job[1]])
	#final turn to init_bearing
	job_des.append('T')
	job_num.append(init_bearing)
	#ending_turn_angle = init_angle - angle_now
	#if ending_turn_angle > 180 :
	#	ending_turn_angle = ending_turn_angle - 360.0
	#job_num.append(ending_turn_angle)

def compass_callback(data):
	global compass_data
	#update compass_data global variable
	compass_data = int(data.data)
	#rospy.loginfo("compass : %s", data.data)

def encoder_callback(data):
	#accumulate encoder data
	global r
	global compass_data
	global job_des
	global job_num
	global x_now
	global y_now
	global dist_travelled

	data_string = data.data
	left_encode, right_encode = data_string.split(" ")
	dist = (float(left_encode) + float(right_encode))/(2.0 * encode_to_mm)
	distpub = '%f %f' % (dist,dist_travelled)
	rospy.loginfo(distpub)
	#FSM of turning
	if(len(job_des) <= 1):
		return 

	if (job_des[0] == 'R') : 	#used for temporally disable the truning part  
                #if (job_des[0] == 'T') :
		#bearing thresholds
		high_threshold = (job_num[0] + 1 + 360) % 360
		low_threshold = (job_num[0] - 1 + 360) % 360

		if (compass_data != low_threshold or compass_data != high_threshold or compass_data != job_num[0]) : #boundary of plus minus 1 degree
			#it is still outside the boundary, continue turning
			d_angle = job_num[0] - compass_data
			if (d_angle > 0) :
				if (d_angle > 180) :
					send_command('L',1)     #test with slowest speed first
				else :
					send_command('R',1)		#after testing will use speed feedback
				send_command('L',1)
			elif (d_angle < 0) :
				if (d_angle < -180) :
					send_command('R',1)
				else :
					send_command('L',1)
		#once turn till target, delete job, considered job done
		if (compass_data == low_threshold or compass_data == high_threshold or compass_data == job_num[0]) :
			send_command('S',0);
			dist_travelled = 0; 	
			del job_des[0]
			del job_num[0]

	#FSM of driving
	elif (job_des[1] == 'F') :
		#accumulate
		dist_travelled = dist_travelled + dist   #this is in mm
		#distance travelled threshold
		dist_threshold = job_num[1] - 0 	#0 mm, I can choose -50mm, but since there will be inefficiencies, 0 error threshold might be good enough
		if (dist_threshold - dist_travelled > 50) :
			send_command('F',3)
		elif (dist_threshold - dist_travelled > 20): 
			send_command('F', 2); 
		elif(dist_threshold - dist_travelled > 3):
			send_command('F', 1);
		if (dist_travelled >= dist_threshold - 3) :
			send_command('S',0)
			rospy.loginfo("Completed a job")
			del job_des[0]
			del job_num[0]

def send_command(command_string, speed):
	global job_des
	global job_num
	#handle the format of the string
	stringToSend = 'S%s00000%dE\n' % (command_string, speed) #might need to add \n behind the E
	#sending the string
	rospy.loginfo(str(stringToSend))
	if ser.isOpen():
		ser.write(stringToSend)
	else:
		rospy.loginfo("Commanding Serial port not connected")

def main_listener():
	rospy.init_node('commander')
	rospy.Subscriber('compass', String, compass_callback)
	rospy.Subscriber('encoder', String, encoder_callback)
	rospy.spin()

if __name__ == '__main__':
	try:
		# AAron's initial one for final testing
		#job_generator(initial_bearing, loops)
		job_generator_straight_1m();
		main_listener()
	except rospy.ROSInterruptException:
		pass
