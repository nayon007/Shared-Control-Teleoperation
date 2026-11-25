from glob import glob

from setuptools import setup

package_name = 'sc_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Md Hafizur Rahman',
    maintainer_email='53692178+nayon007@users.noreply.github.com',
    description='Shared-control teleoperation nodes.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'shared_control = sc_teleop.shared_control:main',
            'dualsense_haptics = sc_teleop.dualsense_haptics:main',
            'franka_wrench_relay = sc_teleop.franka_wrench_relay:main',
            'session_recorder = sc_teleop.session_recorder:main',
        ],
    },
)
