import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
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
            'gz_args': f'-v 4 -r {world_path}'
        }.items()
    )

    #TO DO: Vezi ce e in neregula cu nodurile node_robot_state_publisher, spawn_entity
    # Fara ele, gazebo simulation ruleaza, cu ele, se blocheaza

    # # =========== 2. Pornește Robot State Publisher ===========
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_xml, 'use_sim_time': True }]
    )

     # =========== 3. Spawnează Robotul ===========
    # spawn_entity = TimerAction(
    #     period=3.0,
    #     actions=[Node(
    #         package='ros_gz_sim',
    #         executable='create',
    #         arguments=[
    #             '-world', 'cbrn_world',
    #             '-file', os.path.join(pkg_dir, 'models', 'cbrn_robot', 'model.sdf'),
    #             '-entity', 'cbrn_robot',
    #             '-x', '0', '-y', '0', '-z', '0.5'
    #         ],
    #         output='screen'
    #     )]
    # )


    image_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='image_bridge',
        arguments=[
            "/my_camera@sensor_msgs/msg/Image@gz.msgs.Image",
            "/my_camera/depth@sensor_msgs/msg/Image@gz.msgs.Image",
            "/my_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked"
        ],
        output='screen'
    )


    # =========== 4. Pornește "Puntea" (Bridge) pentru Cameră ===========
    # bridge_node = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     arguments=[
    #         '/my_camera@sensor_msgs/msg/Image[gz.msgs.Image]',
    #         '/my_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo]'
    #     ],
    #     output='screen'
    # )

    # rviz = ExecuteProcess(
    #     cmd=['rviz2', '-d', rviz_config_file],
    #     output='screen'
    # )

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
        image_bridge,
        rviz_node,
        pose_estimator_node,
    ])