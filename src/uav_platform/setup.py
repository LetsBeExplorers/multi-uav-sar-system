from setuptools import find_packages, setup

package_name = 'uav_platform'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='raytracer',
    maintainer_email='ray3618@gmail.com',
    description='Platform abstraction layer for UAV hardware and simulation.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gazebo_driver = uav_platform.gazebo_driver:main',
            'platform_interface = uav_platform.platform_interface:main',
        ],
    },
)
