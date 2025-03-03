#!/bin/bash
# Function to close all terminal windows
close_all_terminals() {
    wmctrl -l | awk '/Terminal/ {print $1}' | xargs -I {} wmctrl -i -c {}
}

# Function to close all windows
close_all_windows() {
    # List all open windows
    wmctrl -l | awk '{print $1}' | xargs -I {} wmctrl -i -c {}
}

# Close all open terminal windows
close_all_terminals

sleep 2

# Open a new terminal (using gnome-terminal, adjust if using a different terminal)
gnome-terminal -- bash -c '
    #Kill old processes
    pkill gzserver
    pkill gzclient

    # Source ROS setup file
    source /opt/ros/noetic/setup.bash
    
    # Export environment variables
    export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models:$GAZEBO_MODEL_PATH
    export GAZEBO_MODEL_PATH=~/ardupilot_gazebo_roscam/src/ardupilot_gazebo/models:$GAZEBO_MODEL_PATH
    export GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:$GAZEBO_PLUGIN_PATH
    export GAZEBO_PLUGIN_PATH=/opt/ros/noetic/lib:$GAZEBO_PLUGIN_PATH
    
    # Source custom workspace setup
    source ~/ardupilot_gazebo_roscam/devel/setup.bash
    
    # Run ROS launch command
    roslaunch ardupilot_gazebo sproj_sim1.launch
    
    # Keep the terminal open
    exec bash
'
sleep 2

# Open new terminals and run sim_vehicle.py commands

gnome-terminal -- bash -c "cd ~/ardupilot/ArduCopter; sim_vehicle.py -v ArduCopter -f gazebo-iris --console --sysid=1 -I0; exec bash"
gnome-terminal -- bash -c "cd ~/ardupilot/ArduCopter; sim_vehicle.py -v ArduCopter -f gazebo-iris --console --sysid=2 -I1; exec bash"
gnome-terminal -- bash -c "cd ~/ardupilot/ArduCopter; sim_vehicle.py -v ArduCopter -f gazebo-iris --console --sysid=3 -I2; exec bash"
gnome-terminal -- bash -c "cd ~/ardupilot/ArduCopter; sim_vehicle.py -v ArduCopter -f gazebo-iris --console --sysid=4 -I3; exec bash"
gnome-terminal -- bash -c "cd ~/ardupilot/ArduCopter; sim_vehicle.py -f gazebo-iris --console -I0; exec bash"


