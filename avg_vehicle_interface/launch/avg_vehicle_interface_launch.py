from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kap_vehicle_interface',
            executable='kap_interface_rpt',
            name='kap_interface_report_node'
        ),
        Node(
            package='kap_vehicle_interface',
            executable='kap_interface_cmd',
            name='kap_interface_command_node'
        )
    ])
