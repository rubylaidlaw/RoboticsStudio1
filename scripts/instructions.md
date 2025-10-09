## Run in Azure terminal:
source /opt/ros/humble/setup.bash
source ~/41068_ws/install/setup.bash
ros2 launch 41068_ignition_bringup 41068_ignition.launch.py slam:=true nav2:=true rviz:=true world:=large_demo

source /opt/ros/humble/setup.bash
source ~/41068_ws/install/setup.bash
python3 ~/41068_ws/src/RoboticsStudio1/scripts/send_goal.py


## Run in WSL terminal:
cd ~/41068_ws
colcon build --symlink-install
source install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch 41068_ignition_bringup 41068_ignition.launch.py slam:=true nav2:=true rviz:=true world:=large_demo

cd ~/41068_ws
colcon build --symlink-install
source install/setup.bash
python3 src/RoboticsStudio1/scripts/send_goal.py





## View topics --> ros2 run rqt_graph rqt_graph

ros2 topic list
ros2 topic echo /map
ros2 topic echo /scan
ros2 topic echo /odom
