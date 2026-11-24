import os
from glob import glob

from setuptools import setup

package_name = 'jcheng_perception_edge'
config_files = sorted(glob(os.path.join('config', '*.yaml')))

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.*')),
        ),
        (os.path.join('share', package_name, 'config'), config_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JCHENG Robotics',
    maintainer_email='maintainer@example.invalid',
    description=(
        'Pi 5 edge: camera capture, JPEG uplink, timestamp matching, depth '
        'back-projection and two-view fusion; no YOLO inference'
    ),
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_edge = jcheng_perception_edge.node:main',
            'perception_fusion = jcheng_perception_edge.fusion_node:main',
            (
                'synthetic_scene_publisher = '
                'jcheng_perception_edge.synthetic:main'
            ),
            'wrist_tf = jcheng_perception_edge.wrist_tf:main',
            'ext_tf = jcheng_perception_edge.ext_tf:main',
        ],
    },
)
