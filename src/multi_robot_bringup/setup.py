from setuptools import setup

package_name = 'multi_robot_bringup'
data_files = [
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
]
# data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/' + package_name + '/worlds', [
    'worlds/turtlebot3_burger_example.wbt', 'worlds/.turtlebot3_burger_example.wbproj',
]))

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch.py']))
data_files.append(('share/' + package_name + '/resource', [
    'resource/turtlebot3_burger_example_map.pgm',
    'resource/turtlebot3_burger_example_map.yaml',
    'resource/turtlebot_webots.urdf',
    'resource/ros2control.yml',
    # 'resource/nav2_params.yaml'
]))
data_files.append(('share/' + package_name + '/worlds', [
    'worlds/turtlebot3_burger_example.wbt', 'worlds/.turtlebot3_burger_example.wbproj',
]))
data_files.append(('share/' + package_name, ['package.xml']))


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='worachit',
    maintainer_email='worachit@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
