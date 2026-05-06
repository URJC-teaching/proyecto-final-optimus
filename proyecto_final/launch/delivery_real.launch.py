# Launch para robot real (Kobuki + Xtion/Astra/OAK).
# Asume kobuki driver, Nav2 y simple_hri arrancados aparte.
# Lanza yolo_bringup + converter YOLO 3D + delivery_node.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Topics por camara. Override con image_topic:=, depth_topic:=, info_topic:=.
_CAMERAS = {
    'xtion': {
        'image':  '/camera/rgb/image_raw',
        'depth':  '/camera/depth_registered/image_raw',
        'info':   '/camera/rgb/camera_info',
        'frame':  'camera_link',
    },
    'astra': {
        'image':  '/camera/color/image_raw',
        'depth':  '/camera/depth/image_raw',
        'info':   '/camera/color/camera_info',
        'frame':  'camera_link',
    },
    'oak': {
        # OAK-D necesita lanzar el driver y la TF estatica aparte
        'image':  '/color/image',
        'depth':  '/stereo/depth',
        'info':   '/stereo/camera_info',
        'frame':  'oak-d_frame',
    },
}


def _build_yolo(context, *args, **kwargs):
    # Lazy: resolvemos yolo_bringup solo si toca (asi el launch importa
    # aunque yolo_bringup no este instalado).
    camera = LaunchConfiguration('camera').perform(context)
    image  = LaunchConfiguration('image_topic').perform(context)
    depth  = LaunchConfiguration('depth_topic').perform(context)
    info   = LaunchConfiguration('info_topic').perform(context)
    frame  = LaunchConfiguration('target_frame').perform(context)
    launch_yolo = LaunchConfiguration('launch_yolo').perform(context).lower() == 'true'
    skip_yolo = LaunchConfiguration('skip_yolo').perform(context).lower() == 'true'

    if skip_yolo or not launch_yolo:
        return []

    preset = _CAMERAS.get(camera)
    if preset is None:
        raise RuntimeError(
            f'camera="{camera}" desconocido. Usa "xtion", "astra", "oak" o pasa '
            'image_topic/depth_topic/info_topic explicitamente.'
        )

    image_t = image if image else preset['image']
    depth_t = depth if depth else preset['depth']
    info_t  = info  if info  else preset['info']
    frame_t = frame if frame else preset.get('frame', 'base_footprint')

    yolo_share = get_package_share_directory('yolo_bringup')
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(yolo_share, 'launch', 'yolo.launch.py')),
            launch_arguments={
                'input_image_topic': image_t,
                'input_depth_topic': depth_t,
                'input_depth_info_topic': info_t,
                'target_frame': frame_t,
                'use_3d': 'True',
                'use_tracking': 'False',
                'use_debug': 'False',
                'depth_image_reliability': '1',
                'depth_info_reliability': '1',
            }.items(),
        )
    ]


def generate_launch_description():
    proj_share = get_package_share_directory('proyecto_final')
    waypoints_default = os.path.join(proj_share, 'config', 'waypoints.yaml')

    # ---------- Argumentos ----------
    waypoints_arg = DeclareLaunchArgument(
        'waypoints_file', default_value=waypoints_default,
        description='YAML con poses (default: waypoints.yaml para mapa de la clase).')

    camera_arg = DeclareLaunchArgument(
        'camera', default_value='xtion',
        description='Preset de camara: xtion, astra o oak.')

    image_topic_arg = DeclareLaunchArgument(
        'image_topic', default_value='',
        description='Override topic RGB. Vacio = usar preset.')
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic', default_value='',
        description='Override topic depth. Vacio = usar preset.')
    info_topic_arg = DeclareLaunchArgument(
        'info_topic', default_value='',
        description='Override topic camera_info. Vacio = usar preset.')

    target_frame_arg = DeclareLaunchArgument(
        'target_frame', default_value='',
        description='Frame para proyectar detecciones 3D. Vacio = usar preset.')

    launch_yolo_arg = DeclareLaunchArgument(
        'launch_yolo', default_value='true',
        description='Lanza yolo_bringup. Pon false si lo lanzas tu en otra terminal.')

    skip_hri_arg = DeclareLaunchArgument(
        'skip_hri', default_value='false',
        description='Salta STT/TTS/Extract/YesNo.')

    skip_yolo_arg = DeclareLaunchArgument(
        'skip_yolo', default_value='false',
        description='No espera receptor en destino, entrega inmediato.')

    forced_dest_arg = DeclareLaunchArgument(
        'forced_dest', default_value='dest1',
        description='Cuando skip_hri o mock_voice activos, qué destino usar.')

    mock_voice_arg = DeclareLaunchArgument(
        'mock_voice', default_value='false',
        description='TTSs reales pero usuario simulado (sin micro).')

    # ---------- Converter YOLO 3D ----------
    yolo_3d_converter = Node(
        package='camera',
        executable='yolo_detection_node_3d',
        name='yolo_detection_node_3d',
        output='screen',
        remappings=[
            ('input_detection', '/yolo/detections_3d'),
            ('output_detection_3d', '/detections_3d'),
        ],
        condition=UnlessCondition(LaunchConfiguration('skip_yolo')),
    )

    # ---------- delivery_node ----------
    delivery_node = Node(
        package='proyecto_final',
        executable='delivery_node',
        name='delivery_node',
        output='screen',
        parameters=[
            LaunchConfiguration('waypoints_file'),
            {
                'skip_hri': LaunchConfiguration('skip_hri'),
                'skip_yolo': LaunchConfiguration('skip_yolo'),
                'forced_dest': LaunchConfiguration('forced_dest'),
                'mock_voice': LaunchConfiguration('mock_voice'),
            },
        ],
    )

    return LaunchDescription([
        waypoints_arg,
        camera_arg,
        image_topic_arg,
        depth_topic_arg,
        info_topic_arg,
        target_frame_arg,
        launch_yolo_arg,
        skip_hri_arg,
        skip_yolo_arg,
        forced_dest_arg,
        mock_voice_arg,
        OpaqueFunction(function=_build_yolo),
        yolo_3d_converter,
        delivery_node,
    ])
