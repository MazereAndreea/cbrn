import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node 
import xacro

def generate_launch_description():

    pkg_dir = get_package_share_directory('sim_env')
    
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'sim.rviz')
    world_path = os.path.join(pkg_dir, 'worlds', 'cbrn_world.world')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'cbrn_robot.urdf.xacro')
    robot_description_xml = xacro.process_file(xacro_file).toxml()

    # =========== 1. Pornește Gazebo (Gz) HEADLESS ===========
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            )
        ]),
        launch_arguments={
            'gz_args': f'-v 4 -r --headless {world_path}'
        }.items()
    )

    #TO DO: Vezi ce e in neregula cu nodurile node_robot_state_publisher, spawn_entity
    # Fara ele, gazebo simulation ruleaza, cu ele, se blocheaza

    # # =========== 2. Pornește Robot State Publisher ===========
    # node_robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='screen',
    #     parameters=[{'robot_description': robot_description_xml, 'use_sim_time': True }]
    # )

    # # =========== 3. Spawnează Robotul ===========
    # spawn_entity = TimerAction(
    #     period=3.0,  # așteaptă 3 secunde
    #     actions=[Node(
    #         package='ros_gz_sim',
    #         executable='create',
    #         arguments=[
    #             '-world', 'cbrn_world',
    #             '-topic', 'robot_description',
    #             '-entity', 'cbrn_robot',
    #             '-x', '0', '-y', '0', '-z', '0.5'
    #         ],
    #         output='screen'
    #     )]
    # )

    # =========== 4. Pornește "Puntea" (Bridge) pentru Cameră ===========
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Folosim 'sensor_msgs/msg/Image' (sintaxa ROS 2 corectă)
            '/my_camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/my_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
        ],
        output='screen'
    )

   
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file], 
        output='screen'
    )

    pose_estimator_node = Node(
        package='cbrn_perception',        
        executable='pose_estimator_node', 
        name='pose_estimator_node',
        output='screen'
    )
    

    return LaunchDescription([
        gz_sim,
        node_robot_state_publisher,
        spawn_entity,
        # bridge_node,
        # rviz_node,
        # pose_estimator_node, # Adaugă nodul de percepție la lista de lansare
    ])