#!/bin/bash

# Function to gracefully kill a process by name
kill_process() {
    process_name=$1
    echo "Attempting to gracefully terminate $process_name..."
    
    # Get the PIDs of the process
    pids=$(pgrep -f $process_name)
    
    if [ -z "$pids" ]; then
        echo "$process_name not running."
    else
        # Send SIGINT (Ctrl+C equivalent) to gracefully terminate
        for pid in $pids; do
            echo "Sending SIGINT to $process_name (PID $pid)..."
            kill -2 $pid
            sleep 1  # Wait a moment for it to terminate
        done

        # Ensure the process is really terminated
        pids=$(pgrep -f $process_name)
        if [ -z "$pids" ]; then
            echo "$process_name terminated successfully."
        else
            echo "$process_name still running, sending SIGKILL..."
            kill -9 $pids  # Force kill
        fi
    fi
}



# Gracefully terminate all SITL (Software In The Loop) terminals
kill_process "ArduCopter"
kill_process "sim_vehicle.py"

sleep 1 

# Gracefully terminate all MAVLink consoles
kill_process "mavproxy.py"
kill_process "mavlink"

sleep 1 
# Gracefully terminate the Gazebo world
kill_process "gzserver"
kill_process "gzclient"

sleep 1

# Wait for everything to be cleaned up
echo "Waiting for all processes to terminate..."
sleep 3

# Additional cleanup, if necessary
echo "All processes terminated. Goodbye!"
