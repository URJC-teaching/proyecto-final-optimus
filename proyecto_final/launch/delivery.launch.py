# Launch low-level: solo delivery_node + converter YOLO 3D.
# Asume Nav2, HRI y yolo_bringup arrancados aparte.
# Para la demo usar delivery_sim.launch.py o delivery_real.launch.py.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('proyecto_final')
    default_waypoints = os.path.join(pkg_share, 'config', 'waypoints.yaml')

    waypoints_arg = DeclareLaunchArgument(
        'waypoints_file',
        default_value=default_waypoints,
        description='YAML con la pose de home y los 2 destinos.'
    )

    target_class_arg = DeclareLaunchArgument(
        'target_class',
        default_value='person',
        description='Clase YOLO a detectar como receptor del paquete.'
    )

    detection_topic_arg = DeclareLaunchArgument(
        'detection_topic',
        default_value='/detections_3d',
        description='Topic de Detection3DArray.'
    )

    # --- 1. Converter YOLO 3D (yolo_msgs -> vision_msgs/Detection3DArray) ---
    yolo_3d_converter = Node(
        package='camera',
        executable='yolo_detection_node_3d',
        name='yolo_detection_node_3d',
        output='screen',
        remappings=[
            ('input_detection', '/yolo/detections_3d'),
            ('output_detection_3d', LaunchConfiguration('detection_topic')),
        ],
    )

    # --- 2. Delivery FSM ---
    delivery_node = Node(
        package='proyecto_final',
        executable='delivery_node',
        name='delivery_node',
        output='screen',
        parameters=[
            LaunchConfiguration('waypoints_file'),
            {
                'detection_topic': LaunchConfiguration('detection_topic'),
                'target_class': LaunchConfiguration('target_class'),
            },
        ],
    )

    return LaunchDescription([
        waypoints_arg,
        target_class_arg,
        detection_topic_arg,
        yolo_3d_converter,
        delivery_node,
    ])
