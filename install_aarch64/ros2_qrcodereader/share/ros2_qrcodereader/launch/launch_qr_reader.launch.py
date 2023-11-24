from launch import LaunchDescription
from launch_ros.actions import Node

#ros2 launch ros2_qrcodereader launch_qr_reader.launch.py 
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_qrcodereader',
            executable='qr_code_reader',
            name='qr_code_reader'
        )
    ])
