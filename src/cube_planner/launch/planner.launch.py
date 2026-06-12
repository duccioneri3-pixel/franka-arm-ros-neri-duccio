# cube_planner/launch/planner.launch.py
import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # URDF da franka_description (percorso corretto)
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

    # SRDF da franka_description (percorso corretto)
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

    return LaunchDescription([
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
    ])