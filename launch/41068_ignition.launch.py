from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

   
    ld = LaunchDescription()

    # Get paths to directories
    pkg_path = FindPackageShare('41068_ignition_bringup')
    config_path = PathJoinSubstitution([pkg_path,
                                       'config'])

    num_foxes_arg = DeclareLaunchArgument(
        'num_foxes',
        default_value='4',
    )
    ld.add_action(num_foxes_arg)
    num_foxes = LaunchConfiguration('num_foxes')

    slam_launch_arg = DeclareLaunchArgument(
        'slam',
        default_value='true',  # or 'false' if you want it off by default
        description='Enable SLAM Toolbox'
    )
    ld.add_action(slam_launch_arg)

    # Additional command line arguments
    # Declare launch arguments for spawn position (dynamic)
    declare_x_arg = DeclareLaunchArgument(
        'spawn_x',
        default_value='0.0',
        description='Spawn X coordinate'
    )
    declare_y_arg = DeclareLaunchArgument(
        'spawn_y',
        default_value='0.0',
        description='Spawn Y coordinate'
    )
    ld.add_action(declare_x_arg)
    ld.add_action(declare_y_arg)

    # Other launch arguments
    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    ld.add_action(use_sim_time_launch_arg)
    
    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description='Flag to launch RViz'
    )
    ld.add_action(rviz_launch_arg)
    
    slam_params_arg = DeclareLaunchArgument(
    'slam_params_file',
    default_value=PathJoinSubstitution([config_path, 'slam_params.yaml']),
    description='Path to the SLAM Toolbox parameters file'
    )
    ld.add_action(slam_params_arg)
    slam_params_file = LaunchConfiguration('slam_params_file')

    nav2_launch_arg = DeclareLaunchArgument(
        'nav2',
        default_value='True',
        description='Flag to launch Nav2'
    )
    ld.add_action(nav2_launch_arg)

    # Get spawn coordinates from launch arguments
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')

    # Get paths to directories
    pkg_path = FindPackageShare('41068_ignition_bringup')
    config_path = PathJoinSubstitution([pkg_path, 'config'])

    # Load robot_description and start robot_state_publisher
    robot_description_content = ParameterValue(
        Command(['xacro ',
                 PathJoinSubstitution([pkg_path, 'urdf', 'husky.urdf.xacro'])]),
        value_type=str)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )
    ld.add_action(robot_state_publisher_node)

    # Publish odom -> base_link transform using robot_localization
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='robot_localization',
        output='screen',
        parameters=[PathJoinSubstitution([config_path,
                                          'robot_localization.yaml']),
                    {'use_sim_time': use_sim_time}]
    )
    ld.add_action(robot_localization_node)

    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',  # for ROS2 Humble/Foxy
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}  # <-- add this
        ],
        condition=IfCondition(LaunchConfiguration('slam'))  # Only if slam:=true
    )
    ld.add_action(slam_node)


    service_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/large_demo/set_pose@ros_gz_interfaces/srv/SetEntityPose',
            '/world/large_demo/create@ros_gz_interfaces/srv/SpawnEntity',
            '/world/large_demo/remove@ros_gz_interfaces/srv/DeleteEntity',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V' # Add this
        ],
        output='screen',
        name='set_pose_bridge'
    )
    ld.add_action(service_bridge)

    # Start Gazebo to simulate the robot in the chosen world
    world_launch_arg = DeclareLaunchArgument(
        'world',
        default_value='simple_trees',
        description='Which world to load',
        choices=['simple_trees', 'large_demo']
    )
    ld.add_action(world_launch_arg)
    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_ign_gazebo'),
                             'launch', 'ign_gazebo.launch.py']),
        launch_arguments={
            'ign_args': [PathJoinSubstitution([pkg_path,
                                               'worlds',
                                               [LaunchConfiguration('world'), '.sdf']]),
                         ' -r']}.items()
    )
    ld.add_action(gazebo)

    # Spawn robot in Gazebo with dynamic spawn_x and spawn_y
    robot_spawner = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-topic', '/robot_description',
                   '-x', spawn_x,
                   '-y', spawn_y,
                   '-z', '0.4']
    )
    ld.add_action(robot_spawner)

    # Bridge topics between gazebo and ROS2
    gazebo_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': PathJoinSubstitution([config_path, 'gazebo_bridge.yaml']),
                     'use_sim_time': use_sim_time}]
    )
    ld.add_action(gazebo_bridge)

    # rviz2 visualises data
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', PathJoinSubstitution([config_path, '41068.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    ld.add_action(rviz_node)

    # Nav2 enables mapping and waypoint following
    nav2 = IncludeLaunchDescription(
        PathJoinSubstitution([pkg_path, 'launch', '41068_navigation.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(LaunchConfiguration('nav2'))
    )
    ld.add_action(nav2)

    spawn_fox = Node(
        package='41068_ignition_bringup',
        executable='foxManagerNode.py',
        output='screen',
        parameters=[
            {'num_foxes': num_foxes},
            {'use_sim_time': use_sim_time}   # <-- add this
        ]
    )
    ld.add_action(spawn_fox)

    #Start fox detector
    fox_detector = Node(
        package='41068_ignition_bringup',
        executable='fox_detector',
        name='fox_detector',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    ld.add_action(fox_detector)

    # Launch the harpoon laser node
    harpoon_laser_node = Node(
        package='41068_ignition_bringup',
        executable='projectile_launch.py',
        name='harpoon_laser_node',
        output='screen',
    )
    ld.add_action(harpoon_laser_node)

    return ld
