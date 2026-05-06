# Helper: lanza Nav2 con el mapa de la clase ya empaquetado en este pkg.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    proj_share = get_package_share_directory('proyecto_final')
    default_map = os.path.join(proj_share, 'maps', 'clase.yaml')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Ruta al fichero .yaml del mapa de Nav2.',
    )

    kobuki_share = get_package_share_directory('kobuki')
    nav_launch = os.path.join(kobuki_share, 'launch', 'navigation.launch.py')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav_launch),
        launch_arguments={'map': LaunchConfiguration('map')}.items(),
    )

    return LaunchDescription([map_arg, nav2])
