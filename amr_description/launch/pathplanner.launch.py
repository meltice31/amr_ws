import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    controller_yaml = os.path.join(get_package_share_directory('amr_description'),'config','controller.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory('amr_description'),'config','bt_navigator.yaml')
    planner_server_yaml = os.path.join(get_package_share_directory('amr_description'),'config','planner_server.yaml')
    recovery_yaml = os.path.join(get_package_share_directory('amr_description'),'config','recovery.yaml')

    return LaunchDescription([
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_server_yaml]
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[controller_yaml]
            
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml]
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[recovery_yaml],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manerger_pathplanner',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names':['planner_server',
                                        'controller_server',
                                        'behavior_server',
                                        'bt_navigator']}]
        )
    ])