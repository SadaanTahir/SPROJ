#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
from utils import make_connections, setmode, arm, takeoff, move, return_to_start, land, disarm, moveglobal

def callback(data):
	print("---------------- RECEIVED DRONE ODOMETRY DATA --------------------- ")
	print(data) 
    # data.pose.pose.position
    # data.pose.pose.orientation
    
def listener():

    rospy.init_node('drone_odometry_listener', anonymous = True)
    rospy.Subscriber("/gazebo/model_states", ModelStates, callback)
    # testing()
    rospy.spin()

def testing():

    #Making connections to all drones
    connections = make_connections()
    
    print("\nACK means 'command has been acknowledged' \n If result = 0 then command successfully executed else it didn't. \n For more details: https://mavlink.io/en/messages/common.html#MAV_RESULT \n")
    #First 4 arguments in conn.mav.command_long_send are target_system, target_component, command_name, 0 if its first transmission of the cmd OR 1-255 if its confirmation of the cmd
    
    # lat = 
    # lon = 

    for i,conn in enumerate(connections):

        if i==2:
            moveglobal(conn,lat,lon,3)
            # return_to_start(conn)
            # conn.close()
            break
    
    # targetsys = conn.target_system
    # targetcomp = conn.target_component


if __name__ == '__main__':
    #listener()
    testing()
