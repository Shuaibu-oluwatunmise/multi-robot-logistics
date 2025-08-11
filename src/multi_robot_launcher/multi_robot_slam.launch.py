from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, GroupAction, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('robot_a_ip', default_value='192.168.8.206'),
        DeclareLaunchArgument('robot_b_ip', default_value='192.168.8.215'),
        DeclareLaunchArgument('robot_a_domain', default_value='6'),
        DeclareLaunchArgument('robot_b_domain', default_value='15'),
        
        # Robot A SLAM Group
        GroupAction([
            # Set environment for Robot A
            SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('robot_a_domain')),
            SetEnvironmentVariable('ROS_DISCOVERY_SERVER', ';;;;;;192.168.8.206:11811;'),
            SetEnvironmentVariable('TURTLEBOT4_NAMESPACE', 'robot_a'),
            
            # Launch SLAM for Robot A
            ExecuteProcess(
                cmd=['bash', '-c', 
                     'export ROS_DOMAIN_ID=6 && ' +
                     'export ROS_DISCOVERY_SERVER=";;;;;;192.168.8.206:11811;" && ' +
                     'ros2 launch turtlebot4_navigation slam.launch.py sync:=false'],
                name='robot_a_slam',
                output='screen',
                prefix='gnome-terminal --tab --title="Robot A SLAM" -- '
            ),
        ], scoped=True),
        
        # Robot B SLAM Group  
        GroupAction([
            # Set environment for Robot B
            SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('robot_b_domain')),
            SetEnvironmentVariable('ROS_DISCOVERY_SERVER', ';;;;;;;;;;;;;;;192.168.8.215:11811;'),
            SetEnvironmentVariable('TURTLEBOT4_NAMESPACE', 'robot_b'),
            
            # Launch SLAM for Robot B
            ExecuteProcess(
                cmd=['bash', '-c', 
                     'export ROS_DOMAIN_ID=15 && ' +
                     'export ROS_DISCOVERY_SERVER=";;;;;;;;;;;;;;;192.168.8.215:11811;" && ' +
                     'ros2 launch turtlebot4_navigation slam.launch.py sync:=false'],
                name='robot_b_slam',
                output='screen',
                prefix='gnome-terminal --tab --title="Robot B SLAM" -- '
            ),
        ], scoped=True),
        
        # Map merge commented out for now - will implement later
        # We'll handle map coordination in the web interface instead
        
        # RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', 'multi_robot_slam.rviz'],
            output='screen'
        ),
    ])
