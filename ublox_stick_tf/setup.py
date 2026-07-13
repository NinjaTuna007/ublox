from setuptools import find_packages, setup

package_name = 'ublox_stick_tf'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Stick Team',
    maintainer_email='your.email@example.com',
    description='UTM/odom TF chain for stick GPS',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'gps_odom_initializer = ublox_stick_tf.gps_odom_initializer:main',
            'gps_tf_publisher = ublox_stick_tf.gps_tf_publisher:main',
        ],
    },
)
