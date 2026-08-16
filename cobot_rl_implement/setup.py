from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'cobot_rl_implement'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'exported_policy'), glob('exported_policy/*.pt')),
        (os.path.join('share', package_name, 'trajectory_points'), glob('trajectory_points/*.json')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),

        ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='muib',
    maintainer_email='muib.akinyele@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cobot_rl = cobot_rl_implement.ros2_policy_node:main',
            'mock_robot = cobot_rl_implement.mock_robot_node:main',
            'json_jogger = cobot_rl_implement.json_jogger:main',
            'pointcloud_jogger = cobot_rl_implement.pointcloud_jogger:main',
        ],
    },
)
