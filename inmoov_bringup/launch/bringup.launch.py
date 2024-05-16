import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    config_file = os.path.join(get_package_share_directory('inmoov_bringup'), 'config', 'config.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file,
            description='Path to the config file'
        ),

        # Commented out hardware-related nodes for later use
        # GroupAction([
        #     Node(
        #         package='micro_ros_agent',
        #         executable='micro_ros_agent',
        #         name='micro_ros_agent_00',
        #         namespace='ns_servobus/00',
        #         output='screen',
        #         arguments=['serial', '--dev', '/dev/ttyACM0']
        #     ),
        # ]),

        # GroupAction([
        #     Node(
        #         package='micro_ros_agent',
        #         executable='micro_ros_agent',
        #         name='micro_ros_agent_01',
        #         namespace='ns_servobus/01',
        #         output='screen',
        #         arguments=['serial', '--dev', '/dev/ttyACM1']
        #     ),
        # ]),

        # GroupAction([
        #     Node(
        #         package='micro_ros_agent',
        #         executable='micro_ros_agent',
        #         name='micro_ros_agent_02',
        #         namespace='ns_servobus/02',
        #         output='screen',
        #         arguments=['serial', '--dev', '/dev/ttyACM2']
        #     ),
        # ]),

        Node(
            package='inmoov_bringup',
            executable='joint_command_dispatcher.py',
            name='joint_command_dispatcher',
            respawn=True
        ),

        Node(
            package='inmoov_bringup',
            executable='joint_status_dispatcher.py',
            name='joint_status_dispatcher',
            respawn=True
        ),

        Node(
            package='inmoov_bringup',
            executable='motor_status_dispatcher.py',
            name='motor_status_dispatcher',
            respawn=True
        ),

        Node(
            package='inmoov_bringup',
            executable='enable_manager.py',
            name='enable_manager',
            respawn=True
        ),

        Node(
            package='inmoov_bringup',
            executable='rviz_manager.py',
            name='rviz_manager',
            respawn=True
        ),
    ])