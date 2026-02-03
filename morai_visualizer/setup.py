from setuptools import setup

package_name = 'morai_visualizer'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', ['launch/visualizer.launch.py']),
    ('share/' + package_name + '/rviz', ['rviz/morai_visualizer.rviz']),
]

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Publish MORAI lane boundaries as RViz markers.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vehicle_marker = morai_visualizer.vehicle_marker_node:main',
            'waypoints_path = morai_visualizer.waypoints_path_node:main',
        ],
    },
)
