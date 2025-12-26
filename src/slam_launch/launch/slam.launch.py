from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import os

def generate_launch_description():

    urdf_path = os.path.join(
        os.getenv('COLCON_PREFIX_PATH').split(':')[0],
        'robot_description/share/robot_description/robot.urdf.xacro'
    )

    return LaunchDescription([
        # Start IMU node early to allow it to calibrate
        Node(
            package='sensors',
            executable='imu_publisher',
            name='imu_publisher',
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_path).read()}]
        ),
        
        Node(
                    package='foxglove_bridge',
                    executable='foxglove_bridge',
                    name='foxglove_bridge',
                    output='screen'
        ),

        # Node that starts after 3 seconds to allow imu to calibrate
        TimerAction(
            period=3.0,  # delay in seconds
            actions=[
                # Node(
                #     package='sensors',
                #     executable='sonar_publisher',
                #     name='sonar_publisher',
                #     output='screen'
                # ),
                Node(
                    package='slam',
                    executable='slam_publisher',
                    name='slam_publisher',
                    output='screen'
                )
            ]
        ),
    ])