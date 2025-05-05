from setuptools import find_packages, setup
import os
from glob import glob 


package_name = 'auto_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nihar',
    maintainer_email='nihar@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autonomous_launch = auto_control.autonomous_launch:main',
            'bomber_launch = auto_control.bomber_launch:main',
            'cpp_auto_launch = auto_control.cpp_auto_launch:main',
            'drone_manual_launch = auto_control.drone_manual_launch:main',
            'program_esc = auto_control.program_esc:main',
            'servo_test = auto_control.servo_test:main',
            'Servo_Test = auto_control.Servo_Test:main',
            'test_launch = auto_control.test_launch:main',
            'updated_launch = auto_control.updated_launch:main',
        ],
    },
)
