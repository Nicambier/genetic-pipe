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
    urdf = get_package_share_directory('pipebot_description')+'/urdf/pipebot.urdf'
    
    params = {'robot_description': open(urdf).read()}
    
    control = Node(
        package='pipebot_control',
        executable='net_control',
        output='screen',
        #remappings=[
            #('cmd_vel', '/pipebot/cmd_vel'),
            #('laser_scan', '/pipebot/laser_scan')
        #]
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    
    robot_spawner = Node(package='gazebo_ros', executable='spawn_entity.py', arguments=['-topic', 'robot_description', '-entity', LaunchConfiguration('rb_id'), '-robot_namespace', LaunchConfiguration('rb_id')], output='screen')
        
    return LaunchDescription([
        DeclareLaunchArgument(
            'rb_id', default_value='bot1',
            description='Specify robot identifier'
        ),
        PushRosNamespace(LaunchConfiguration(variable_name='namespace', default=LaunchConfiguration('rb_id'))),
        
        control,
        robot_state_publisher,
        robot_spawner
    ])


 #<node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false"
    #output="screen" ns="/rrbot" args="joint1_position_controller joint2_position_controller joint_state_controller"/>

  #<!-- convert joint states to TF transforms for rviz, etc -->
  #<node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"
    #respawn="false" output="screen">
    #<remap from="/joint_states" to="/rrbot/joint_states" />
  #</node>
