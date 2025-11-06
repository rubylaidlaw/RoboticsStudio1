#!/bin/bash

# Source ROS 2 setup
source /opt/ros/humble/setup.bash
source ~/41068_ws/install/setup.bash

sleep 2

# Find and kill all ign gazebo/gazebo/ignition/gzserver/gzclient related PIDs
for pattern in "ign gazebo" "gazebo" "ignition" "gzserver" "gzclient"; do
    pids=$(ps aux | grep -i "$pattern" | grep -v grep | awk '{print $2}')
    if [ ! -z "$pids" ]; then
        echo "Killing PIDs for pattern '$pattern': $pids"
        kill $pids
        sleep 2
        # Force kill if still running
        for pid in $pids; do
            if ps -p $pid > /dev/null; then
                echo "Force killing $pid"
                kill -9 $pid
            fi
        done
    fi
done

# Also kill any lingering ros2 nodes
pkill -f ros2

sleep 2

# Optionally: clear robot localization state file
LOCALIZATION_STATE_FILE=~/install/41068_ignition_bringup/config/robot_localization_state.yaml
if [ -f "$LOCALIZATION_STATE_FILE" ]; then
    echo "Removing localization state file: $LOCALIZATION_STATE_FILE"
    rm $LOCALIZATION_STATE_FILE
fi

# Optionally: backup your RViz config if desired
RVIZ_CONFIG=~/install/41068_ignition_bringup/share/41068_ignition_bringup/config/41068.rviz
if [ -f "$RVIZ_CONFIG" ]; then
    echo "Backing up RViz config"
    cp $RVIZ_CONFIG $RVIZ_CONFIG.backup
fi

echo "All simulation processes killed and state cleared."
