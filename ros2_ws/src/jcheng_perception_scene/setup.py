import glob

from setuptools import find_packages, setup

package_name = 'jcheng_perception_scene'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/schemas',
         glob.glob('schemas/*.json')),
    ],
    install_requires=['setuptools', 'numpy', 'PyYAML', 'opencv-python-headless'],
    zip_safe=True,
    maintainer='JCHENG Robotics',
    maintainer_email='maintainer@example.invalid',
    description=(
        'Offline ore instance segmentation, versioned scene state, '
        'evidence-labelled evaluation and CLI'
    ),
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ore_scene = jcheng_perception_scene.cli:main',
        ],
    },
)
