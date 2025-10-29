#!/bin/bash

sleep 2

pkill -f gazebo
pkill -f ignition

sleep 2

LOCALIZATION_STATE_FILE=~/41068_ws/install/41068_ignition_bringup/config/robot_localization_state.yaml
if [ -f "$LOCALIZATION_STATE_FILE" ]; then
  rm $LOCALIZATION_STATE_FILE
fi

RVIZ_CONFIG=~/41068_ws/install/41068_ignition_bringup/share/41068_ignition_bringup/config/41068.rviz
if [ -f "$RVIZ_CONFIG" ]; then
  cp $RVIZ_CONFIG $RVIZ_CONFIG.backup
fi

pkill -f ros2
ros2 launch 41068_ignition_bringup 41068_ignition.launch.py &