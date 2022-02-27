#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Twist
import tf
from tf import transformations
import math

##### Ahmet Gokce       150180076
##### Mehmet Karaaslan  150180053

## CAK
## high_speed_controller.py
##
## BLG456E Assignment 2 skeleton
##
## Instructions: Edit the control loop below.
## Read further instructions
##
## 

waypoint=None

#waypoint callback
def waypoint_callback(msg): #  callback

    #***************************************
    #***          Obtain current destination
    #***************************************

    #save waypoint data for printing out in main loop
    global waypoint
    waypoint=msg;


if __name__ == '__main__':

    #setup ROS node, subscribe waypoint_cb to the topic /waypoint_cmd & publish motor commands
    rospy.init_node("schumacher_456")
    waypoint_subscriber = rospy.Subscriber("/waypoint_cmd", Transform, waypoint_callback) # <--- set up callback
    motor_command_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=100)
    #you could in principle also subscribe to the laser scan as is done in assignment 1.

    #setup transform cache manager
    listener = tf.TransformListener()

    #start a loop; one loop per two second
    delay = rospy.Rate(0.5); # perhaps this could be faster for a controller?
    while not rospy.is_shutdown():


        #***************************************
        #***          Obtain current robot pose
        #***************************************
        
        try:
            #grab the latest available transform from the odometry frame (robot's original location - usually the same as the map unless the odometry becomes inaccurate) to the robot's base.
            
            (translation,orientation) = listener.lookupTransform("/odom", "/base_footprint", rospy.Time(0));
        except  (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("EXCEPTION:",e)
            #if something goes wrong with this just go to bed for a second or so and wake up hopefully refreshed.
            delay.sleep()
            continue
        

        #***************************************
        #***          Print current robot pose
        #***************************************

        #Print out the x,y coordinates of the transform
        print("Robot is believed to be at (x,y): (",translation[0],",",translation[1],")")

        #Convert the quaternion-based orientation of the latest message to Euler representation in order to get z axis rotation
        r_xorient, r_yorient, r_zorient = transformations.euler_from_matrix(transformations.quaternion_matrix(orientation))
        robot_theta = r_zorient  # only need the z axis
        print("Robot is believed to have orientation (theta): (",robot_theta,")\n")

        #***************************************
        #***          Print current destination
        #***************************************

        # the waypoint variable is filled in in the waypoint_callback function above, which comes from incoming messages
        # subscribed to in the .Subscriber call above.

        #Print out the x,y coordinates of the latest message
        print("Current waypoint (x,y): (",waypoint.translation.x,",",waypoint.translation.y,")")

        #Convert the quaternion-based orientation of the latest message to angle-axis in order to get the z rotation & print it.
        waypointrotq = [waypoint.rotation.x,waypoint.rotation.y,waypoint.rotation.z,waypoint.rotation.w]
        w_xorient, w_yorient, w_zorient = transformations.euler_from_matrix(transformations.quaternion_matrix(waypointrotq))
        waypoint_theta=w_zorient # only need the z axis
        print("Current waypoint (theta): (",waypoint_theta,")\n")

        #***************************************
        #***          DRIVE THE ROBOT HERE (same as with assignment 1 
        #**           - except you are driving towards a goal not away from an obstacle)
        #***************************************

        #for containing the motor commands to send to the robot
        motor_command=Twist()

        dx = waypoint.translation.x - translation[0] # X distance between waypoint and current positon
        dy = waypoint.translation.y - translation[1] # Y distance between waypoint and current positon

        p = math.hypot(dx, dy) # distance between waypoint and robot -> distance error
        r = math.atan2(dy, dx) # rotational degree of direction of waypoint vector
        q = r - robot_theta    # rotational difference between waypoint's pointing direction and robot's pointing direction
        
        #Used P control on linear velocity
        #Cosine is used for reducing speed before turning for smoothness
        motor_command.linear.x=0.3*p*math.cos(abs(q))

        # according to rotational difference, set rotating speed. 
        # if rotational difference close to 0, rotating speed close to 0.
        motor_command.angular.z= 0.5 * q 

        #print("Current linear velocity is: ", motor_command.linear.x, "and angular velocity is: ", motor_command.angular.z, "\n")

        motor_command_publisher.publish(motor_command)
        
        delay.sleep()
        # we don't need to call spinOnce like in roscpp as callbacks happen in different threads
    
    
    print("ROS shutdown now I will also go to sleep. I hope I didn't crash. Night night.")
