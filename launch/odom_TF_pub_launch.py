from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='main_strategy',
            namespace='krabi_ns',
            executable='odometry_node',
            name='odom_tf'
            #,remappings=[
            #    ('/input/pose', '/turtlesim1/turtle1/pose'),
            #    ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            #]
            ,parameters=[
            {"init_pose/x": 0.0},
            {"init_pose/y": 0.0},
            {"init_pose/theta": 0.0},
            {"publish_tf_odom": True}
        ]
        )
    ])