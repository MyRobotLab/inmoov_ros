import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch.conditions import IfCondition

def generate_launch_description():
    # Get the path to the package
    pkg_inmoov_description = FindExecutable.find('inmoov_description')
    
    # Declare the launch arguments
    declare_model_arg = DeclareLaunchArgument('model', default_value='')
    declare_gui_arg = DeclareLaunchArgument('gui', default_value='False')

    # Set up the robot description
    robot_description = {
        'robot_description': Command([
            PathJoinSubstitution([FindExecutable.find('xacro'), 'xacro ']),
            PathJoinSubstitution([pkg_inmoov_description, 'robots', 'inmoov.urdf.xacro'])
        ])
    }

    # Nodes
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'source_list': ['rviz_command'], 'rate': 20}],
        condition=IfCondition(LaunchConfiguration('gui'))
    )
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher'
    )
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', PathJoinSubstitution([pkg_inmoov_description, 'urdf.rviz'])],
        required=True
    )

    # Return the launch description
    return LaunchDescription([
        declare_model_arg,
        declare_gui_arg,
        robot_description,
        joint_state_publisher_node,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
