#!/usr/bin/env python3
## Ahmet Gökçe 	150180076
## Mehmet Karaaslan 	150180053
## Emre Güler 		040150342


import math
import os
import rospy
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Twist
from geometry_msgs.msg import *
import tf
import math
from gazebo_msgs.srv import DeleteModel,SpawnModel
from listener import Listener
from tf import transformations

import roslib.packages

def spawn_model(model_name):
   
    initial_pose = Pose()
    initial_pose.position.x = 0
    initial_pose.position.y = 0
    #initial_pose.position.y = (int(product_number)-1)* 0.25 if we would delivered different points 

    # Spawn the new model #~/.gazebo/models
    model_path = '/'+'home/'+os.getlogin()+'/.gazebo'+'/models/'
    model_xml = ''

    with open (model_path + model_name + '/model.sdf', 'r') as xml_file:
        model_xml = xml_file.read().replace('\n', '')

    spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
    spawn_model_prox('product', model_xml, '', initial_pose, 'world')


delete_model_prox = rospy.ServiceProxy('gazebo/delete_model',DeleteModel)

state = "Listening"
rooms = ["1","2","3","4","5","6","7","8"]

room1 = [3.76,2]
room2 = [2.89,2]
room3 = [2,2]
room4 = [1.14,2]

room5 = [3.76,0]
room6 = [2.89,0]
room7 = [2,0]
room8 = [1.14,0]


#product1 = [[0.5,1], [3.5,1], [3.7,1.4],[3.76,2], [3.7,1.4], [3.5,1], [0.5,1], [0.5,0],[0,0]]
#product2 = [[0.5,1], [2,1], [2.7,1.17],[2.89,2], [2.7,1.17], [2,1], [0.5,1], [0.5,0.25],[0,0.25]]
#product3 = [[0.5,1], [1.81,1], [2,1.5], [2,2], [2,1.5], [1.81,1], [0.5,1], [0.5,0.5],[0,0.5]]
#product4 = [[0.5,1], [1,1], [1.14, 1.5], [1.14,2], [1.14,1.5], [1,1], [0.5,1], [0.5,0.75],[0,0.75]]

#product5 = [[0.5,1], [3.5,1], [3.7,0.6],[3.76,0], [3.7,0.6], [3.5,1], [0.5,1], [0.5,1],[0,1]]
#product6 = [[0.5,1], [2,1], [2.7,0.83],[2.89,0], [2.7,0.83], [2,1], [0.5,1], [0.5,1.25],[0,1.25]]
#product7 = [[0.5,1], [1.81,1], [2,0.5], [2,0], [2,0.5], [1.81,1], [0.5,1], [0.5,1.5],[0,1.5]]
#product8 = [[0.5,1], [1,1], [1.14, 0.5], [1.14,0], [1.14,0.5], [1,1], [0.5,1], [0.5,1.75],[0,1.75]]

product1 = [[0,1], [3.5,1], [3.7,1.4],[3.76,2], [3.7,1.4], [3.5,1], [0.5,1], [0,0]]
product2 = [[0,1], [2,1], [2.7,1.17],[2.89,2], [2.7,1.17], [2,1], [0.5,1], [0,0]]
product3 = [[0,1], [1.81,1], [2,1.5], [2,2], [2,1.5], [1.81,1], [0.5,1], [0,0]]
product4 = [[0,1], [1,1], [1.14, 1.5], [1.14,2], [1.14,1.5], [1,1], [0.5,1], [0,0]]

product5 = [[0,1], [3.5,1], [3.7,0.6],[3.76,0], [3.7,0.6], [3.5,1], [0.5,1], [0,0]]
product6 = [[0,1], [2,1], [2.7,0.83],[2.89,0], [2.7,0.83], [2,1], [0.5,1], [0,0]]
product7 = [[0,1], [1.81,1], [2,0.5], [2,0], [2,0.5], [1.81,1], [0.5,1], [0,0]]
product8 = [[0,1], [1,1], [1.14, 0.5], [1.14,0], [1.14,0.5], [1,1], [0.5,1], [0,0]]

path =[]
counter = 0
waypoint_x = 0
waypoint_y = 0
q_prev = 0


if __name__ == '__main__':
	if state=="Listening":#take voice input from the user
		print('Give me a room number.')
		listener = Listener()
		listener.start()

	rospy.init_node('referee')
	
	#rospy.wait_for_service("/gazebo/get_model_state")
	#rospy.wait_for_service("/gazebo/apply_body_wrench")
	motor_command_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=100)
	waypoint_pub = rospy.Publisher('/waypoint_cmd',Transform,queue_size=1)
	    #setup transform cache manager
	    
	delay = rospy.Rate(1.0); 
	tf_listener = tf.TransformListener()

	while not rospy.is_shutdown():
	
		motor_command=Twist()
		try:
			(translation,orientation) = tf_listener.lookupTransform("/odom", "/base_footprint", rospy.Time(0));
		except  (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
			print("EXCEPTION:",e)
			#if something goes wrong with this just go to bed for a second or so and wake up hopefully refreshed.
			delay.sleep()
			continue

		temp_room = listener.room 

		if (temp_room in rooms) and (state =="Listening"):
			listener.stop = True
			state = "Moving"
			if temp_room == "1":
				path = product1
			elif temp_room == "2":
				path = product2
			elif temp_room == "3":
				path = product3
			elif temp_room == "4":
				path = product4
			elif temp_room == "5":
				path = product5
			elif temp_room == "6":
				path = product6
			elif temp_room == "7":
				path = product7
			elif temp_room == "8":
				path = product8
			
			waypoint_x = path[counter][0]
			waypoint_y = path[counter][1]
			
			try:
				delete_model_prox("product")
			except:
				pass

		r_xorient, r_yorient, r_zorient = transformations.euler_from_matrix(transformations.quaternion_matrix(orientation))
		robot_theta = r_zorient  # only need the z axis

		dx = waypoint_x - translation[0]
		dy = waypoint_y - translation[1]
		
		new_x = math.cos(robot_theta) * dx + math.sin(robot_theta) * dy #new plane found with rotation matrix
		new_y = math.cos(robot_theta) * dy - math.sin(robot_theta) * dx

		p = math.hypot(new_x, new_y) #distance between robot and waypoint
		q = math.atan2(new_y, new_x) #angle between robot and waypoint

		if p<0.1 and state == "Moving":
			
			print("Reached waypoint:",waypoint_x,waypoint_y)
			if temp_room == "1" and [waypoint_x,waypoint_y] == room1:
				delete_model_prox("product1")
				print(f"Product {temp_room} is grabbed")
				print("Reached to room ", temp_room)
			elif temp_room == "2" and [waypoint_x,waypoint_y] == room2:
				delete_model_prox("product2")
				print(f"Product {temp_room} is grabbed")
				print("Reached to room ", temp_room)
			elif temp_room == "3" and [waypoint_x,waypoint_y] == room3:
				delete_model_prox("product3")
				print(f"Product {temp_room} is grabbed")
				print("Reached to room ", temp_room)
			elif temp_room == "4" and [waypoint_x,waypoint_y] == room4:
				delete_model_prox("product4")
				print(f"Product {temp_room} is grabbed")
				print("Reached to room ", temp_room)
			elif temp_room == "5" and [waypoint_x,waypoint_y] == room5:
				delete_model_prox("product5")
				print(f"Product {temp_room} is grabbed")
				print("Reached to room ", temp_room)
			elif temp_room == "6" and [waypoint_x,waypoint_y] == room6:
				delete_model_prox("product6")
				print(f"Product {temp_room} is grabbed")
				print("Reached to room ", temp_room)
			elif temp_room == "7" and [waypoint_x,waypoint_y] == room7:
				delete_model_prox("product7")
				print(f"Product {temp_room} is grabbed")
				print("Reached to room ", temp_room)
			elif temp_room == "8" and [waypoint_x,waypoint_y] == room8:
				delete_model_prox("product8")
				print(f"Product {temp_room} is grabbed")
				print("Reached to room ", temp_room)
			
			counter = counter + 1
			if(counter == len(path)):
				print("Finished")
				spawn_model('product')
				state = "Listening"
				listener.stop = False
				temp_room =""
				listener.room = None
				counter = 0
				path = []

			else:
				waypoint_x = path[counter][0]
				waypoint_y = path[counter][1]
		if(state =="Moving"):
			#print("Robot is believed to be at (x,y): (",translation[0],",",translation[1],")")
			#print("Robot is going to (x,y): (",waypoint_x,",",waypoint_y,")")
			if p>1: #max dist for linear speed
				p = 1
			motor_command.linear.x=0.4*p*math.cos(abs(q))
			motor_command.angular.z=0.9*q + 0.5*(q-q_prev)
			q_prev = q
			#print("linear speed x is:",motor_command.linear.x)
			#print("angular speed z is:",motor_command.angular.z)
			motor_command_publisher.publish(motor_command)
		else:
			motor_command.linear.x=0
			motor_command.angular.z=0
			#print("linear speed x is:",motor_command.linear.x)
			#print("angular speed z is:",motor_command.angular.z)
			motor_command_publisher.publish(motor_command)

	print("ROS shutdown now I will also go to sleep. I hope I didn't crash. Night night.")









