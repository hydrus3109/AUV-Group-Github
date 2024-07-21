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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shamakgowda',
    maintainer_email='shamak.gowda@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'publisher = intro_to_ros.publisher:main',
            'subscriber = intro_to_ros.subscriber:main',
            #'subscriber = intro_to_ros.bluerov2_sensors:main',
            'drivepub = intro_to_ros.rovdrive:main',
            'arm = intro_to_ros.arm:main',
            'disarm = intro_to_ros.disarm:main',
            'mode = intro_to_ros.mode:main'
        ],
    },
)
