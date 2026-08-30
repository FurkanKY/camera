"""Mikroskop etap kurulumu: 5 webcam + IP kamera + mikroskop kamera."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


MICROSCOPE_DEVICE = (
    '/dev/v4l/by-id/'
    'usb-Vimicro_Co._ltd_Vimicro_USB2.0_UVC_PC_Camera-video-index0'
)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('microscope_device', default_value=MICROSCOPE_DEVICE),
        Node(
            package='cameranew_pkg', executable='camera_manager', name='camera_manager',
            output='screen',
        ),
        Node(
            package='cameranew_pkg', executable='webcam', name='microscope_publisher',
            output='screen', parameters=[{
                'camera_index': LaunchConfiguration('microscope_device'),
                'topic': '/microscope/image_compressed',
                'frame_id': 'microscope',
                'width': 640, 'height': 480, 'fps': 30,
                'jpeg_quality': 75, 'input_fourcc': 'YUYV',
            }],
        ),
    ])
