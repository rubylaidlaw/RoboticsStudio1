## Launching Harpoon URDF 

 ```bash
  cd ros2_ws
  colcon build
  source install/setup.bash
  ```
New Terminal:

 ```bash
  ros2 run robot_state_publisher robot_state_publisher ~/ros2_ws/src/RoboticsStudio1/urdf/harpoon.urdf
  ros2 run rviz2 rviz2
  ```


 ```bash
  cd ros2_ws
  colcon build --symlink-install
  source ~/ros2_ws/install/setup.bash
  ros2 launch 41068_ignition_bringup 41068_ignition.launch.py slam:=true nav2:=true rviz:=true
  ros2 node list

  ```

