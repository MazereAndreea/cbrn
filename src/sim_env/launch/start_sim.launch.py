import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node 
from launch.substitutions import Command
import xacro
import subprocess
import tempfile

def urdf_to_sdf(robot_description_xml):
    # 2. Creează fișier temporar URDF
    with tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.urdf') as urdf_file:
        urdf_file.write(robot_description_xml)
        urdf_path = urdf_file.name

    # 3. Convertește URDF → SDF (Gazebo Harmonic acceptă doar SDF)
    sdf_xml = subprocess.check_output([
        "gz", "sdf", "-p", urdf_path   # "-p" = convert to SDF and print
    ]).decode("utf-8")
    return sdf_xml

def generate_launch_description():

    pkg_dir = get_package_share_directory('sim_env')
    
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'sim.rviz')
    world_path = os.path.join(pkg_dir, 'worlds', 'cbrn_world.world')
    xacro_file = os.path.join(pkg_dir, 'description', 'cbrn_robot.urdf.xacro')
    robot_description_xml = xacro.process_file(
        xacro_file,
        mappings={'use_sdf': 'true', 'sim_mode': 'true'}
    ).toxml()
    robot_description_sdf = urdf_to_sdf(robot_description_xml)
    sdf_path = os.path.join(pkg_dir, 'description', 'cbrn_robot.sdf')
    with open(sdf_path, 'w') as sdf_file:
        sdf_file.write(robot_description_sdf)

    # =========== 1. Pornește Gazebo (Gz) HEADLESS ===========
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            )
        ]),
        launch_arguments={
            'gz_args': f'-v 4 -r {world_path}'
        }.items()
    )

    params = {'robot_description': robot_description_xml}
    # # =========== 2. Pornește Robot State Publisher ===========
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    #  # =========== 3. Spawnează Robotul ===========
    spawn_entity = TimerAction(
        period=3.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-file', sdf_path,
                '-entity', 'cbrn_robot',
                '-x', '0', '-y', '0', '-z', '0.5'
            ],
            output='screen'
        )]
    )


    # image_bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     name='image_bridge',
    #     arguments=[
    #         "/camera@sensor_msgs/msg/Image@gz.msgs.Image",
    #     ],
    #     output='screen'
    # )


    # =========== 4. Pornește "Puntea" (Bridge) pentru Cameră ===========

    bridge_params = os.path.join(
        pkg_dir,
        'config',
        'camera.yaml'
    )   
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{"config_file": bridge_params}] #don't change this
    )
    

    # rviz = ExecuteProcess(
    #     cmd=['rviz2', '-d', rviz_config_file],
    #     output='screen'
    # )

    rviz_node = TimerAction(
        period = 13.0,
        actions = [
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                parameters=[{'use_sim_time': True}],
                arguments=['-d', rviz_config_file], 
                output='screen'
            )
        ]
    )

    diff_drive_spawner = TimerAction(
        period=10.0, # Așteptăm 10 secunde să fim siguri că robotul e în lume
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["diff_drive_controller"],
            )
        ]
    )

    joint_broad_spawner = TimerAction(
        period=10.0, # Același delay
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
            )
        ]
    )
    
    return LaunchDescription([
        gz_sim,
        node_robot_state_publisher,
        spawn_entity,
        # image_bridge,
        bridge_node,
        rviz_node,
        diff_drive_spawner,
        joint_broad_spawner
    ])