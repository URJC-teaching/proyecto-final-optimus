import os
from glob import glob
from setuptools import setup

package_name = 'proyecto_final'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.yaml') + glob('maps/*.pgm')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Angel Ruiz',
    maintainer_email='a.ruizf.2022@alumnos.urjc.es',
    description='Proyecto final Robotica - Repartidor autonomo (Nav2 + HRI + YOLO)',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'delivery_node = proyecto_final.delivery_node:main',
        ],
    },
)
