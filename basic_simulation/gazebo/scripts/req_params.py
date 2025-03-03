#!/usr/bin/env python
import sys
import time
from pymavlink import mavutil

port = 14550
conn = mavutil.mavlink_connection(f"udpin:localhost:{port}")
conn.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (conn.target_system, conn.target_component))


# Request all parameters
conn.mav.param_request_list_send(conn.target_system, conn.target_component)
while True:
    time.sleep(0.01)
    try:
        message = conn.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
        print('name: %s\tvalue: %d' % (message['param_id'], message['param_value']))
    except Exception as error:
        print(error)
        sys.exit(0)