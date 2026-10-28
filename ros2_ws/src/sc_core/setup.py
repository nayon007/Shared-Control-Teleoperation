from setuptools import setup

package_name = 'sc_core'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Md Hafizur Rahman',
    maintainer_email='53692178+nayon007@users.noreply.github.com',
    description='ROS-independent core of the shared-control teleoperation project.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'offline_sim = sc_core.offline_sim:main',
        ],
    },
)
