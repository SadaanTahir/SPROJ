from pymavlink import mavutil
from utils import connect_n_drones


# Establish connections with Drone 1,2
connections = connect_n_drones(2)
drone1 = connections[0]
drone2 = connections[1]

# Wait for heartbeats
drone1.wait_heartbeat()
drone2.wait_heartbeat()

def wait_for_msg(drone):
    while True:
        # Wait for any MAVLink message
        msg = drone.recv_match(blocking=True)

        if msg:
            # if the received message is a string
            if msg.get_type() == "STATUSTEXT":
                # Extract and print the text message
                custom_message = msg.text.decode('utf-8')
                print(f"Received message: {custom_message}")
                return

            
            # if the received message is an array
            if msg.get_type() == "ATT_POS_MOCAP":
                # Extract and print the data from the message
                custom_array = [msg.q[0], msg.q[1], msg.q[2]]
                print(f"Received array: {custom_array}")
                return
        

# Sending a custom string message from Drone 2 to Drone 1
custom_message = "Hello from Drone 1!"
drone1.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, custom_message.encode('utf-8'))
print('Message sent from Drone 1 to Drone 2')

wait_for_msg(drone2)

#Sending an array
custom_array = [12.4, 3.8, 9.6]
# Encode the array into a MAVLink message (e.g., as part of a POSITION_TARGET or custom message)
# Here we are using an ATT_POS_MOCAP message, which has fields to carry such information
drone1.mav.att_pos_mocap_send(0, *custom_array, 0, 0, 0)
print("Array sent from Drone 1 to Drone 2")

wait_for_msg(drone2)

#received message can be of any custom message type as well ...