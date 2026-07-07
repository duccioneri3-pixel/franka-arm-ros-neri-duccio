# bt_demo.launch.py — come full_demo, ma lancia planner_bt (Behavior Tree)
# al posto di planner_node. Stesso contesto MoveIt (URDF/SRDF/kinematics).
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import yaml
from launch.substitutions import Command, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    set_libgl = SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1')
    set_mesa = SetEnvironmentVariable('MESA_GL_VERSION_OVERRIDE', '3.3')

    # Usa la NOSTRA copia del launch di simulazione, con allowed_start_tolerance
    # rilassato (0.05) per l'assestamento del braccio simulato. L'originale del
    # dottorando resta intatto in franka_gazebo_bringup.
    gazebo_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('cube_planner'),
                'launch',
                'moveit_gazebo_custom_tolerance.launch.py'
            )
        )
    )

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

    kinematics_path = os.path.join(
        get_package_share_directory('franka_fr3_moveit_config'),
        'config', 'kinematics.yaml'
    )
    with open(kinematics_path, 'r') as f:
        kinematics = yaml.safe_load(f)

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

    # Behavior Tree planner al posto del planner lineare
    cube_planner_bt = TimerAction(
        period=35.0,
        actions=[
            Node(
                package='cube_planner',
                executable='planner_bt',
                name='cube_planner_bt',
                parameters=[
                    {'use_sim_time': True},
                    robot_description,
                    robot_description_semantic,
                    kinematics,
                    {'transport_offset': LaunchConfiguration('transport_offset')},
                ],
                output='screen',
            )
        ]
    )

    declare_transport = DeclareLaunchArgument(
        'transport_offset', default_value='0.45',
        description='Offset verticale di trasporto (default 0.45; usare valore assurdo per forzare il recovery)'
    )

    return LaunchDescription([
        declare_transport,
        set_libgl,
        set_mesa,
        gazebo_moveit,
        cube_detector,
        cube_planner_bt,
    ])
