from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch configuration for world file
    world_file = LaunchConfiguration('world', default='empty.sdf')
    
    # Path to ros_gz_sim launch files
    ros_gz_share = FindPackageShare('ros_gz_sim')
    
    # 1️⃣ Launch Ignition Gazebo
 
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': 'empty.sdf -r'
        }.items()
    )


    # 2️⃣ Spawn the robot entity
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', PathJoinSubstitution([FindPackageShare('41068_ignition_bringup'), 'models', 'box.sdf']),
            '-name', 'box',
            '-x', '0',
            '-y', '0',
            '-z', '0.5'
        ],
        output='screen'
    )

    # 3️⃣ Bridge /cmd_vel and /odom
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/world/empty/create@ros_gz_interfaces/srv/SpawnEntity@ignition.msgs.EntityFactory@ignition.msgs.Boolean',
            '/world/empty/remove@ros_gz_interfaces/srv/DeleteEntity@ignition.msgs.Entity@ignition.msgs.Boolean'
    
        ],
        output='screen'
    )

    spawn_box = Node(
        package='41068_ignition_bringup',
        executable='box.py',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='empty.sdf',
            description='Ignition world file'
        ),
        gazebo,
        # spawn_entity,
        bridge,
        spawn_box
    ])
