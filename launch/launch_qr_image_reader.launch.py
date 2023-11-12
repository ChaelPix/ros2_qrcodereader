from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_share = get_package_share_directory('ros2_qrcodereader')
    default_image_path = [pkg_share, '/imgs/bonjourros2.png']

    image_path_arg = DeclareLaunchArgument(
        'image_path',
        default_value=default_image_path,
        description='Path to the QR code image')

    image_path = LaunchConfiguration('image_path')

    return LaunchDescription([
        image_path_arg,
        Node(
            package='ros2_qrcodereader',
            executable='qr_code_reader',
            name='qr_code_reader'
        ),
        Node(
            package='ros2_qrcodereader',
            executable='qr_image_publisher',
            name='qr_image_publisher',
            parameters=[{'image_path': image_path}]
        )
    ])
