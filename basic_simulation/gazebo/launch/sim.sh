#!/bin/bash

# Open new terminals and run sim_vehicle.py commands

gnome-terminal -- bash -c "cd ~/ardupilot/ArduCopter; sim_vehicle.py -v ArduCopter -f gazebo-iris --console --sysid=1 -I0; exec bash"
gnome-terminal -- bash -c "cd ~/ardupilot/ArduCopter; sim_vehicle.py -v ArduCopter -f gazebo-iris --console --sysid=2 -I1; exec bash"
gnome-terminal -- bash -c "cd ~/ardupilot/ArduCopter; sim_vehicle.py -v ArduCopter -f gazebo-iris --console --sysid=3 -I2; exec bash"
gnome-terminal -- bash -c "cd ~/ardupilot/ArduCopter; sim_vehicle.py -v ArduCopter -f gazebo-iris --console --sysid=4 -I3; exec bash"
gnome-terminal -- bash -c "cd ~/ardupilot/ArduCopter; sim_vehicle.py -f gazebo-iris --console -I0; exec bash"
 
