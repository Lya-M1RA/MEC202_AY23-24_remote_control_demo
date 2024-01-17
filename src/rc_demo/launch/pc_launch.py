from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    joy_node = Node(
        package='joy',  # 替换为 joy_node 所在的包名
        executable='joy_node',  # 替换为 joy_node 的可执行文件名
        name='joy_node'
    )

    disp_fov_node = Node(
        package='video_view',
        executable='video_viewer_node',
        name='video_viewer_node',
        parameters=[
            {'image_topic': '/image_raw'},
            {'image_transport': 'theora'}
        ],
        output='screen'
    )


    return LaunchDescription([
        joy_node,
        disp_fov_node
    ])
