from setuptools import setup

package_name = 'jcheng_perception_core'

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
    ],
    install_requires=['setuptools', 'numpy', 'PyYAML'],
    zip_safe=True,
    maintainer='JCHENG Robotics',
    maintainer_email='maintainer@example.invalid',
    description=(
        'ROS-free timestamp matching, depth back-projection, association, '
        'distance-weighted two-view fusion and Kalman tracking'
    ),
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
