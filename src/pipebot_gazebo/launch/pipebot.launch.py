from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_path = get_package_share_directory('gazebo_ros')
    world = get_package_share_directory('pipebot_gazebo')+'/worlds/pipe_obstacle.world'
    urdf = get_package_share_directory('pipebot_description')+'/urdf/pipebot.urdf'
    control = get_package_share_directory('pipebot_control')
        
    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value=world,
            description='Specify world file name'
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gazebo_path, '/launch/gazebo.launch.py'])
        ),
        
        #LAUNCH BOT WITH CONTROLLER
        IncludeLaunchDescription(PythonLaunchDescriptionSource([control, '/launch/pipebot_control.launch.py'])),
        
    ])
