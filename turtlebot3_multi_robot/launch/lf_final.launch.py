import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import OpaqueFunction


def generate_launch_description():
    ld = LaunchDescription()

    # Declare number of followers
    declare_num_followers = DeclareLaunchArgument(
        'num_followers', 
        default_value='3', 
        description='Number of follower robots'
    )
    
    num_followers = LaunchConfiguration('num_followers')

    # Declare leader namespace
    declare_leader_namespace = DeclareLaunchArgument(
        'leader_namespace', 
        default_value='tb1', 
        description='Namespace for leader robot'
    )
    
    leader_namespace = LaunchConfiguration('leader_namespace')

    # Declare follower namespaces dynamically
    follower_namespace_args = [
        DeclareLaunchArgument(
            f'follower_namespace_{i+1}', 
            default_value=f'tb{i+1}', 
            description=f'Namespace for follower robot {i+1}'
        )
        for i in range(1, 4)  # Defaulting to 3 followers
    ]

    follower_namespaces = [
        LaunchConfiguration(f'follower_namespace_{i+1}') for i in range(1, 4)
    ]

    # Declare threshold distance
    declare_th_distance = DeclareLaunchArgument(
        'threshold_distance',
        default_value='1.2',  
        description='Threshold distance between leader and follower'
    )
    
    threshold_distance = LaunchConfiguration('threshold_distance')

    # Include simulation launch file
    simulation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_multi_robot'), 
                'launch', 
                'gazebo_multi_nav2_world.launch.py'
            )
        )
    )

    # Add launch arguments
    ld.add_action(declare_num_followers)
    ld.add_action(declare_leader_namespace)
    ld.add_action(declare_th_distance)
    for arg in follower_namespace_args:
        ld.add_action(arg)

    # Add simulation launch
    ld.add_action(simulation_cmd)

    # Function to spawn followers
    def spawn_followers(context):
        num_followers_value = int(context.launch_configurations['num_followers'])  # Convert string to int
        nodes = []
        for i in range(num_followers_value):
            node = Node(
                package='leader_follower',
                namespace=follower_namespaces[i],
                executable='lf_control',
                output='screen',
                parameters=[{
                    'leader_namespace': leader_namespace,
                    'follower_namespace': follower_namespaces[i]
                }]
            )
            nodes.append(node)
        return nodes

    # Delay follower spawning to ensure Gazebo has started
    spawn_followers_action = TimerAction(
        period=7.0,  # Wait for 5 seconds (adjust if needed)
        actions=[OpaqueFunction(function=spawn_followers)]
    )

    ld.add_action(spawn_followers_action)

    return ld
