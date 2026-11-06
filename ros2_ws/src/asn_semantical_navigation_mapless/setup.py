from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'asn_semantical_navigation_mapless'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='master-jan',
    maintainer_email='jan.cuesta@icloud.com',
    description='Reactive mapless semantic navigation prototype',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': [
            "mapless_motion_node = asn_semantical_navigation_mapless.mapless_motion_node:main",
            "nbv_auto_node = asn_semantical_navigation_mapless.nbv_auto_node:main",
            "approach_goal_node = asn_semantical_navigation_mapless.approach_goal_node:main",
            "object_localizer_node = asn_semantical_navigation_mapless.object_localizer_node:main",
            "search_command_node = asn_semantical_navigation_mapless.search_command_node:main",
            "yolo_detector_node = asn_semantical_navigation_mapless.yolo_detector_node:main",
            "semantic_user_interface_node = asn_semantical_navigation_mapless.semantic_user_interface_node:main",
            "semantical_mission_manager_node = asn_semantical_navigation_mapless.semantical_mission_manager_node:main"
    ]},
)
