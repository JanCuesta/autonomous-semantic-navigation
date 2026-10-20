from setuptools import find_packages, setup

package_name = 'asn_semantical_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='master-jan',
    maintainer_email='jan.cuesta@icloud.com',
    description='Coverage mapping and confidence-guided object search',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'coverage_node = asn_semantical_navigation.coverage_node:main',
            'nbv_preview_node = '
            'asn_semantical_navigation.nbv_preview_node:main',
            'nbv_execute_once_node = '
            'asn_semantical_navigation.nbv_execute_once_node:main',
            'nbv_auto_node = asn_semantical_navigation.nbv_auto_node:main',
            'search_command_node = '
            'asn_semantical_navigation.search_command_node:main',
            'yolo_detector_node = '
            'asn_semantical_navigation.yolo_detector_node:main',
        ],
    },
)
