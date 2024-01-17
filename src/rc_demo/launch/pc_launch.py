from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    joy_node = Node(
        package='joy',  # 替换为 joy_node 所在的包名
        executable='joy_node',  # 替换为 joy_node 的可执行文件名
        name='joy_node'
    )

    trans_fov_node = Node(
        package='image_transport',
        executable='republish',
        name='republish_theora',
        remappings=[
            ('in', '/image_raw/theora'),
            ('out', '/rc_demo/fov_image')
        ],
        parameters=[
            {'in_transport': 'theora'},
            {'out_transport': 'raw'}
        ]
    )


    return LaunchDescription([
        joy_node,
        trans_fov_node
    ])
