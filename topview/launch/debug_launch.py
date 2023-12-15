from launch import LaunchDescription
from launch_ros.actions import Node
front_high=0
front_low=2
behind_high=4
behind_low=6
def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rqt_image_view",
            executable="rqt_image_view"
        ),
        Node(
            package="v4l2_camera",
            namespace=f"video{front_high}",
            executable="v4l2_camera_node",
            remappings=[
            ("image_raw",f"/video{front_high}/image_raw"),
            ("camera_info",f"video{front_high}/camera_info"),
            ("image_raw/compressed",f"video{front_high}/compressed")
            ],
            parameters=[
            {"video_device":f"/dev/video{front_high}"},
            {"image_size":[320,240]}
            ]
        ),
        Node(
            package="v4l2_camera",
            namespace=f"video{front_low}",
            executable="v4l2_camera_node",
            remappings=[
            ("image_raw",f"/video{front_low}/image_raw"),
            ("camera_info",f"video{front_low}/camera_info"),
            ("image_raw/compressed",f"video{front_low}/compressed")
            ],
            parameters=[
            {"video_device":f"/dev/video{front_low}"},
            {"image_size":[320,240]}
            ]
        ),
        Node(
            package="v4l2_camera",
            namespace=f"video{behind_high}",
            executable="v4l2_camera_node",
            remappings=[
            ("image_raw",f"/video{behind_high}/image_raw"),
            ("camera_info",f"video{behind_high}/camera_info"),
            ("image_raw/compressed",f"video{behind_high}/compressed")
            ],
            parameters=[
            {"video_device":f"/dev/video{behind_high}"},
            {"image_size":[320,240]}
            ]
        ),
        Node(
            package="v4l2_camera",
            namespace=f"video{behind_low}",
            executable="v4l2_camera_node",
            remappings=[
            ("image_raw",f"/video{behind_low}/image_raw"),
            ("camera_info",f"video{behind_low}/camera_info"),
            ("image_raw/compressed",f"video{behind_low}/compressed")
            ],
            parameters=[
            {"video_device":f"/dev/video{behind_low}"},
            {"image_size":[320,240]}
            ]
        ),
        Node(
        package='image_transport',
        executable='republish',
        arguments=["raw"],
        remappings=[('in', '/image_front'),
                    ('out', '/front'),
                    ('out/compressed','/image_front/compressed')
                   ]
        ),
        Node(
        package='image_transport',
        executable='republish',
        arguments=["raw"],
        remappings=[('in', '/image_behind'),
                    ('out', 'behind'),
                    ('out/compressed','/image_behind/compressed')
                   ]
        ),
        Node(
        package='topview',
        executable='camera_node',
        parameters=[{"front_high","/video0/image_raw"},
                    {"front_low","/video2/image_raw"},
                    {"behind_high","/video4/image_raw"},
                    {"behind_low","/video6/image_raw"}
            ]
        
        )
    ])
