from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    joy_node = Node(
        package='joy',  # 替换为 joy_node 所在的包名
        executable='joy_node',  # 替换为 joy_node 的可执行文件名
        name='joy_node'
    )

    display_fov_node = Node(
        package='rc_demo',  # 替换为 manual_mode 节点所在的包名
        executable='display_fov',  # 替换为 manual_mode 的可执行文件名
        name='display_fov'
    )

    return LaunchDescription([
        joy_node,
        display_fov_node
    ])
