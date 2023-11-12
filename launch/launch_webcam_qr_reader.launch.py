from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_qrcodereader')
    d_frequency = '100'  

    frequency_arg = DeclareLaunchArgument(
        'frequency', 
        default_value=d_frequency,
        description='Frequency Rate of Webcam images'
    )

    frequency = LaunchConfiguration('frequency')

    return LaunchDescription([
        frequency_arg,
        Node(
            package='ros2_qrcodereader',
            executable='qr_code_reader',
            name='qr_code_reader'
        ),
        Node(
            package='ros2_qrcodereader',
            executable='webcam_publisher',
            name='webcam_publisher',
            parameters=[{'frequency': frequency}]
        )
    ])
