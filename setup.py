from setuptools import find_packages, setup

package_name = 'intro_to_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['mavros_launch.py']),
    ],
    install_requires=['setuptools','rclpy','mavros_msgs', 'numpy', 'mavros'],
    zip_safe=True,
    maintainer='aidan',
    maintainer_email='agao3019@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['publisher = intro_to_ros.publisher:main',
                            'subscriber = intro_to_ros.subscriber:main',
                            'subscriber2 = intro_to_ros.bluerov2_sensors:main',
                            'physicssubscriber = intro_to_ros.physics_sim:main',
                            'drivepub = intro_to_ros.rovdrive:main',
                            'armdisarm = intro_to_ros.armdisarm:main'
        ],
    },
)
