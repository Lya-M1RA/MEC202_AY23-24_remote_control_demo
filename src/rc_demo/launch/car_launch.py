from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        parameters=[{'image_width': 1280, 'image_height': 720}]
    )

    wheel_control_node = Node(
        package='rc_remote',
        executable='wheel_control',
        name='wheel_control'
    )

    return LaunchDescription([
        usb_cam_node,
        wheel_control_node
    ])
