from setuptools import find_packages, setup

package_name = 'turtlebot_ws'

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
    maintainer='cashwn',
    maintainer_email='cashwn@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = turtlebot_ws.publisher_member_function:main',
            'map_publisher = turtlebot_ws.map_publisher:main',
            'costmap_publisher = turtlebot_ws.costmap_publisher:main',
        ],
    },
)
