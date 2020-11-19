from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    gazebo_path = get_package_share_directory('gazebo_ros')
    simulation = get_package_share_directory('pipebot_gazebo')
    
    genetic_algo = Node(
        package='pipebot_genetic',
        executable='GA_client',
        output='screen',
    )
        
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Set to "false" to run headless.'),
        
        IncludeLaunchDescription(PythonLaunchDescriptionSource([simulation, '/launch/pipebot.launch.py'])),
        genetic_algo
    ])
