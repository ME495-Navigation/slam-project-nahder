""" Loads the turtlebot3_burger URDF and allows it to be viewed in rviz. """
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution
import launch

def generate_launch_description():
    
    return LaunchDescription([
        
    DeclareLaunchArgument(
        name='color',
        default_value="purple",
        choices=['red', 'blue', 'purple', 'green'],
        description='Color choice for the turtle bot. purple by default'
    ),
    
    # use_jsp launch argument
    DeclareLaunchArgument(
        "use_jsp",
        default_value="true", choices=['true', 'false'],
        description="Choose whether to use the joint state publisher or not(true/false). \
        true by default"
    ),

    # use_rviz launch argument
    DeclareLaunchArgument(
        "use_rviz",
        default_value="true", choices=['true', 'false'],
        description="Choose whether to use rviz or not (true/false). \
        true by default"
    ),
    
    # joint_state_publisher node
    Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration("use_jsp"), "true")),
    ),
    
    # robot_state_publisher node, loading in the turtlebot3_burger urdf
    Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": 
                Command([ExecutableInPackage("xacro", "xacro"), " ",
                        PathJoinSubstitution(
                    [FindPackageShare("nuturtle_description"), \
                     "urdf/turtlebot3_burger.urdf.xacro"])]
            )}
        ]
    ),
    
    # rviz2 node
    Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration("use_rviz"), "true")),
        on_exit=launch.actions.Shutdown()
    )
    
    ])