from setuptools import find_packages, setup

package_name = 'drone_sensing'

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
    maintainer='aidan',
    maintainer_email='aidansun05@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "altitude_publisher = drone_sensing.altitude:main",
            "line_detection = drone_sensing.line_detection:main",
            "optical_flow_subscriber = drone_sensing.optical_flow_subscriber:main",
            "ar_tag_detecter = drone_sensing.ar_tag_detection:main",
            "stereo_camera = drone_sensing.stereo_camera:main",
            "localization = drone_sensing.localization:main",
            "pickup_dropoff = drone_sensing.pickup_dropoff:main"
        ],
    },
)
