# Launch all-in-one para simulacion: Gazebo + Nav2 + YOLO + delivery.
# simple_hri se lanza aparte (vive en su propio venv).

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    proj_share = get_package_share_directory('proyecto_final')
    kobuki_share = get_package_share_directory('kobuki')

    waypoints_default = os.path.join(proj_share, 'config', 'waypoints_sim.yaml')

    # ---------- Argumentos ----------
    waypoints_arg = DeclareLaunchArgument(
        'waypoints_file', default_value=waypoints_default,
        description='YAML con poses de home y los 2 destinos.')

    launch_gazebo_arg = DeclareLaunchArgument(
        'launch_gazebo', default_value='true',
        description='Lanza Gazebo + Kobuki simulado.')

    launch_nav2_arg = DeclareLaunchArgument(
        'launch_nav2', default_value='true',
        description='Lanza Nav2 con el mapa aws_house.')

    launch_yolo_arg = DeclareLaunchArgument(
        'launch_yolo', default_value='true',
        description='Lanza yolo_bringup con topics simulados.')

    skip_hri_arg = DeclareLaunchArgument(
        'skip_hri', default_value='false',
        description='Salta STT/TTS/Extract/YesNo (util si simple_hri no esta).')

    skip_yolo_arg = DeclareLaunchArgument(
        'skip_yolo', default_value='false',
        description='No espera receptor en destino, entrega inmediato.')

    forced_dest_arg = DeclareLaunchArgument(
        'forced_dest', default_value='dest1',
        description='Cuando skip_hri o mock_voice activos, qué destino usar.')

    mock_voice_arg = DeclareLaunchArgument(
        'mock_voice', default_value='false',
        description='TTSs reales pero usuario simulado (no necesita micro). '
                    'Util para demo en simulación sin micrófono.')

    # En sim el robot arranca en (0,0): publicamos /initialpose para no tener
    # que hacer 2D Pose Estimate en RViz cada vez.
    auto_initial_pose_arg = DeclareLaunchArgument(
        'auto_initial_pose', default_value='true',
        description='Publicar /initialpose en home al arrancar (sim only).')

    # Args de visualizacion (en VNC poner gui:=false rviz:=False camera:=false)
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Mostrar GUI de Gazebo. En VNC poner false.')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='True',
        description='Mostrar RViz. En VNC poner False.')

    camera_arg = DeclareLaunchArgument(
        'camera', default_value='true',
        description='Camara simulada del Kobuki. false reduce carga en VNC.')

    # ---------- 1. Gazebo + Kobuki sim ----------
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(kobuki_share, 'launch', 'simulation.launch.py')),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'camera': LaunchConfiguration('camera'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_gazebo')),
    )

    # ---------- 2. Nav2 con mapa aws_house ----------
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(kobuki_share, 'launch', 'navigation_sim.launch.py')),
        launch_arguments={
            'rviz': LaunchConfiguration('rviz'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_nav2')),
    )

    # ---------- 3. YOLO bringup (topics simulados) ----------
    # Lazy: solo resolvemos yolo_bringup si launch_yolo=true (asi no peta
    # en PCs sin el paquete instalado). skip_yolo tambien lo desactiva.
    def _build_yolo_bringup(context):
        launch_yolo = LaunchConfiguration('launch_yolo').perform(context).lower() == 'true'
        skip_yolo = LaunchConfiguration('skip_yolo').perform(context).lower() == 'true'
        if skip_yolo or not launch_yolo:
            return []
        yolo_share = get_package_share_directory('yolo_bringup')
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(yolo_share, 'launch', 'yolo.launch.py')),
            launch_arguments={
                'input_image_topic': '/rgbd_camera/image',
                'input_depth_topic': '/rgbd_camera/depth_image',
                'input_depth_info_topic': '/rgbd_camera/camera_info',
                'target_frame': 'camera_link',
                'use_3d': 'True',
                'use_tracking': 'False',
                'use_debug': 'False',
            }.items(),
        )]
    yolo_bringup = OpaqueFunction(function=_build_yolo_bringup)

    # ---------- 4. Converter YOLO 3D ----------
    yolo_3d_converter = Node(
        package='camera',
        executable='yolo_detection_node_3d',
        name='yolo_detection_node_3d',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('input_detection', '/yolo/detections_3d'),
            ('output_detection_3d', '/detections_3d'),
        ],
        condition=UnlessCondition(LaunchConfiguration('skip_yolo')),
    )

    # ---------- 5. delivery_node ----------
    delivery_node = Node(
        package='proyecto_final',
        executable='delivery_node',
        name='delivery_node',
        output='screen',
        parameters=[
            LaunchConfiguration('waypoints_file'),
            {
                'use_sim_time': True,
                'skip_hri': LaunchConfiguration('skip_hri'),
                'skip_yolo': LaunchConfiguration('skip_yolo'),
                'forced_dest': LaunchConfiguration('forced_dest'),
                'mock_voice': LaunchConfiguration('mock_voice'),
                'auto_initial_pose': LaunchConfiguration('auto_initial_pose'),
            },
        ],
    )

    return LaunchDescription([
        waypoints_arg,
        launch_gazebo_arg,
        launch_nav2_arg,
        launch_yolo_arg,
        skip_hri_arg,
        skip_yolo_arg,
        forced_dest_arg,
        mock_voice_arg,
        auto_initial_pose_arg,
        gui_arg,
        rviz_arg,
        camera_arg,
        gazebo_launch,
        nav2_launch,
        yolo_bringup,
        yolo_3d_converter,
        delivery_node,
    ])
