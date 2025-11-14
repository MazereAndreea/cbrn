import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # Calea către pachetul tău
    pkg_dir = get_package_share_directory('sim_env')

    # Calea către fișierul .world
    world_file = os.path.join(pkg_dir, 'worlds', 'cbrn_test.world')

    # Calea către fișierul URDF/Xacro
    xacro_file = os.path.join(pkg_dir, 'urdf', 'cbrn_robot.urdf.xacro')

    # Procesează fișierul Xacro pentru a obține XML-ul URDF
    robot_description_xml = xacro.process_file(xacro_file).toxml()

    # =========== 1. Pornește Gazebo (Gz) ===========
    # Folosim fișierul de lansare standard al ros_gz_sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
        # Trimitem fișierul .world ca argument
        launch_arguments={'gz_args': world_file}.items(),
    )

    # =========== 2. Pornește Robot State Publisher ===========
    # Acesta citește URDF-ul și publică transformările (TF) ale robotului
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_xml,
            'use_sim_time': True # FOARTE IMPORTANT: Folosește timpul simulării!
        }]
    )

    # =========== 3. Adaugă (Spawn) Robotul în Lume ===========
    # Folosim nodul "create" din ros_gz_sim
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description', # Sursa URDF-ului
            '-entity', 'cbrn_robot',       # Numele robotului în Gz
            '-x', '0',                     # Poziția de start X
            '-y', '0',                     # Poziția de start Y
            '-z', '0.5'                    # Poziția de start Z (să nu cadă prin podea)
        ],
        output='screen'
    )

    # =========== 4. Pornește "Puntea" (Bridge) ===========
    # Conectează topicul Gz al camerei la un topic ROS 2
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/my_camera' # Numele topicului de bază al camerei din .xacro
        ],
        output='screen'
    )

    # Returnează lista de acțiuni de lansat
    return LaunchDescription([
        gz_sim,
        node_robot_state_publisher,
        spawn_entity,
        bridge_node,
    ])