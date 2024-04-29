from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros.actions
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
def generate_launch_description():
    isBlue_value = LaunchConfiguration('isBlue')
    xRobotPos_value = LaunchConfiguration('xRobotPos')
    yRobotPos_value = LaunchConfiguration('yRobotPos')
    zRobotOrientation_value = LaunchConfiguration('zRobotOrientation')
    
    isBlue_launch_arg = DeclareLaunchArgument(
        'isBlue',
        default_value='False'
    )
    xRobotPos_launch_arg = DeclareLaunchArgument(
        'xRobotPos',
        default_value='1.25'
    )
    yRobotPos_launch_arg = DeclareLaunchArgument(
        'yRobotPos',
        default_value='0.5'
    )
    zRobotOrientation_launch_arg = DeclareLaunchArgument(
        'zRobotOrientation',
        default_value='0.0'
    )
    return LaunchDescription([
        isBlue_launch_arg, xRobotPos_launch_arg, yRobotPos_launch_arg, zRobotOrientation_launch_arg,
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
            {"init_pose/x": xRobotPos_value},
            {"init_pose/y": yRobotPos_value},
            {"init_pose/theta": zRobotOrientation_value},
            {"publish_tf_odom": True}
        ]
        )
    ])