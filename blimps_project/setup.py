from setuptools import find_packages, setup

package_name = 'blimps_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/blimps_project/launch', ['launch/blimp_launch.py']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='prajjwal',
    maintainer_email='prajjwal@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'camera = blimps_project.camera_node:main',
        	'blob = blimps_project.blob_detection:main',
        	'servo_controller = blimps_project.servo_controller:main',
            'turtle_controller = blimps_project.turtle_controller:main',
        ],
    },
)
