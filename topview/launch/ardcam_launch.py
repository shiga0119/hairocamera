from launch import LaunchDescription
from launch_ros.actions import Node
flont=0
behind=2
right=4
left=6
def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rqt_image_view",
            executable="rqt_image_view"
        ),
        Node(
            package="v4l2_camera",
            namespace=f"video{flont}",
            executable="v4l2_camera_node",
            remappings=[
            ("image_raw",f"/video{flont}/image_raw"),
            ("camera_info",f"video{flont}/camera_info")
            ],
            parameters=[
            {"video_device":f"/dev/video{flont}"},
            {"image_size":[320,240]}
            ]
        ),
        Node(
            package="v4l2_camera",
            namespace=f"video{behind}",
            executable="v4l2_camera_node",
            remappings=[
            ("image_raw",f"/video{behind}/image_raw"),
            ("camera_infof",f"video{behind}/camera_info")
            ],
            parameters=[
            {"video_device":f"/dev/video{behind}"},
            {"image_size":[320,240]}
            ]
        ),
        Node(
            package="v4l2_camera",
            namespace=f"video{right}",
            executable="v4l2_camera_node",
            remappings=[
            ("image_raw",f"/video{right}/image_raw"),
            ("camera_infof",f"video{right}/camera_info")
            ],
            parameters=[
            {"video_device":f"/dev/video{right}"},
            {"image_size":[320,240]}
            ]
        ),
        Node(
        package='image_transport',
        executable='republish',
        arguments=["raw"],
        remappings=[('in', '/image_tpv'),
                    ('out', '/image_raw/compressed')
                   ]
        )
    ])
