from setuptools import find_packages, setup

package_name = 'track_on_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/track_camera_hand.launch.py']),
        ('share/' + package_name + '/config', ['config/config.yaml']),
    ],
    install_requires=['setuptools', 'rclpy', 'tf2_ros', 'sensor_msgs', 'cv_bridge', 'pyyaml', 'numpy<2'],
    zip_safe=True,
    maintainer='root1',
    maintainer_email='root1@todo.todo',
    description='ROS2节点：实时摄像头关键点跟踪',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'track_camera_hand_node = track_on_ros2.track_camera_hand_node:main',
        ],
    },
)
