import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import yaml
from launch.substitutions import Command, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    set_libgl = SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1')
    set_mesa  = SetEnvironmentVariable('MESA_GL_VERSION_OVERRIDE', '3.3')

    # Gazebo + MoveIt
    gazebo_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('franka_gazebo_bringup'),
                'launch',
                'moveit_gazebo_franka_arm_example_controller.launch.py'
            )
        )
    )

    # URDF
    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', 'fr3', 'fr3.urdf.xacro'
    )
    robot_description = {
        'robot_description': ParameterValue(
            Command([
                FindExecutable(name='xacro'), ' ', franka_xacro_file,
                ' ros2_control:=false',
                ' hand:=true',
                ' robot_type:=fr3',
                ' arm_prefix:=',
                ' robot_ip:=dont-care',
                ' use_fake_hardware:=true',
                ' fake_sensor_commands:=false',
            ]),
            value_type=str
        )
    }

    # SRDF
    franka_srdf_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', 'fr3', 'fr3.srdf.xacro'
    )
    robot_description_semantic = {
        'robot_description_semantic': ParameterValue(
            Command([
                FindExecutable(name='xacro'), ' ', franka_srdf_file,
                ' hand:=true',
                ' arm_prefix:=',
            ]),
            value_type=str
        )
    }

    # Kinematics
    kinematics_path = os.path.join(
        get_package_share_directory('franka_fr3_moveit_config'),
        'config', 'kinematics.yaml'
    )
    with open(kinematics_path, 'r') as f:
        kinematics = yaml.safe_load(f)

    # Detector — dopo 25s
    cube_detector = TimerAction(
        period=30.0,
        actions=[
            Node(
                package='cube_detector',
                executable='detector_node',
                name='cube_detector',
                parameters=[{'use_sim_time': True}],
                output='screen',
            )
        ]
    )

    # Planner C++ — dopo 30s (5s dopo il detector)
    cube_planner = TimerAction(
        period=35.0,
        actions=[
            Node(
                package='cube_planner',
                executable='planner_node',
                name='cube_planner',
                parameters=[
                    {'use_sim_time': True},
                    robot_description,
                    robot_description_semantic,
                    kinematics,
                ],
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        set_libgl,
        set_mesa,
        gazebo_moveit,
        cube_detector,
        cube_planner,
    ])
