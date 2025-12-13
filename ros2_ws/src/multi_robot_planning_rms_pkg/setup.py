import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'multi_robot_planning_rms_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'config'), glob('config/*yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel García Burgos, Hongfan Yang, José Antonio García Campanario, José Manuel Parejo Alonso, José Francisco López Ruiz',
    maintainer_email='josloprui6@alum.us.es',
    description='Paquete principal para el sistema de planificación multi-robot',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'planning_node = multi_robot_planning_rms_pkg.planning_node:main',
            'darp_node = multi_robot_planning_rms_pkg.darp_node:main',
            'control_node = multi_robot_planning_rms_pkg.control_node:main',
            'px4_transform_node = multi_robot_planning_rms_pkg.px4_transform_node:main',
            'visualization_node = multi_robot_planning_rms_pkg.visualization_node:main'
        ],
    },
)