import os
from glob import glob

from setuptools import setup

package_name = 'jcheng_perception_gpu'
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
        'Laptop GPU host: asynchronous two-stream YOLO detection with '
        'capture-timestamp preservation and bounded per-camera queues'
    ),
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_gpu = jcheng_perception_gpu.node:main',
        ],
    },
)
