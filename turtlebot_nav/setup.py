from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'turtlebot_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # information on launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cashwn',
    maintainer_email='cashwn@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'costmap_publisher = turtlebot_nav.costmap_publisher:main',
            'sub_robot_position = turtlebot_nav.sub_robot_position:main',
        ],
    },
)
