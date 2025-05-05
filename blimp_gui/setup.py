from setuptools import setup
#import glob

package_name = 'blimp_gui'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ["resource/" + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/rosbridge_camera_launch.py']),
        ('share/' + package_name + '/static', ['static/camera_view.html'] ),
        ('share/' + package_name + '/static', ['static/index.html'] ),
        ('lib/' + package_name, ['scripts/export_graph.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nihar Masurkar',
    maintainer_email='nmasurka@asu.edu',
    description='Web GUI for visualizing OAK-D Lite camera and TF using ROS 2 and rosbridge',
    entry_points={
        'console_scripts': [
            'webserver = blimp_gui.webserver:main',
        ],
    },
)

