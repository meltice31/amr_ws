import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    controller_yaml = os.path.join(get_package_share_directory('amr_description'), 'config', 'controller.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory('amr_description'), 'config', 'bt_navigator.yaml')
    planner_yaml = os.path.join(get_package_share_directory('amr_description'), 'config', 'planner_server.yaml')
    recovery_yaml = os.path.join(get_package_share_directory('amr_description'), 'config', 'recovery.yaml')
    nav2_yaml = os.path.join(get_package_share_directory('amr_description'), 'config', 'amcl_config.yaml')
    map_file = os.path.join(get_package_share_directory('amr_description'), 'maps', 'sim_map.yaml')
    
    return LaunchDescription([     
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True}, 
                        {'yaml_filename':map_file}]
        ),
            
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml]
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml]),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml]),
            
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[recovery_yaml],
            output='screen'),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['map_server',
                                        'amcl',
                                        'controller_server',
                                        'planner_server',
                                        'behavior_server',
                                        'bt_navigator']}])
    ])