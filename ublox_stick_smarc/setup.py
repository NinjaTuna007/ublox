from setuptools import find_packages, setup

package_name = 'ublox_stick_smarc'

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
    description='Publish SMARC GPS topics from ublox_dgnss fix and VELNED',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'stick_smarc_publisher = ublox_stick_smarc.stick_smarc_publisher:main',
            'nmea_serial_gnss = ublox_stick_smarc.nmea_serial_gnss:main',
        ],
    },
)
