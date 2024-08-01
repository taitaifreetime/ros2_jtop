from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    ros2_jtop_pkg = get_package_share_directory('ros2_jtop')
    rqt_gui_setting = os.path.join(ros2_jtop_pkg, 'rqt_gui_setting', 'jtop.perspective')

    return LaunchDescription([
        Node(
            package='ros2_jtop',
            namespace='',
            executable="jtop",
            output='screen',
            name='jtop',
            parameters=[{
                'interval':0.5,
                'max_num_cpus':8
            }],
            emulate_tty=True,
        ), 
        Node(
            package='rqt_gui',
            namespace='',
            executable="rqt_gui",
            output='screen',
            name='jtop_gui',
            emulate_tty=True,
            arguments=['--perspective-file', rqt_gui_setting]
        )
    ])
