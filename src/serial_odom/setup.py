from setuptools import find_packages, setup

package_name = 'serial_odom'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
    'setuptools',
    'pyserial',
	],

    zip_safe=True,
    maintainer='ksvsd',
    maintainer_email='ksvsd@example.com',  # <-- replace with real email if desired
    description='Reads odometry data from serial and publishes as a ROS 2 Odometry message.',
    license='MIT',  # <-- change if using a different license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_odom_node = serial_odom.serial_odom_node:main',
        ],
    },
)

