from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='cashier_system', executable='bill_generator'),
        Node(package='cashier_system', executable='inventory_manager'),
        Node(package='cashier_system', executable='status_viewer'),
    ])
