from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'cbrn_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andreea',
    maintainer_email='andreeamazere.am@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller = cbrn_perception.robot_controller:main',
            'mediapipe_detector = cbrn_perception.mediapipe_detector:main',
            'yolo_detector = cbrn_perception.yolo_detector:main',
            'movenet_detector = cbrn_perception.movenet_detector:main',
            'ResNet50_detector = cbrn_perception.ResNet50_detector:main',
        ],
    },
)
