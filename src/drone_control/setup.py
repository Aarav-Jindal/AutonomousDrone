from setuptools import find_packages, setup

package_name = 'drone_control'

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
            "arm_disarm_node = drone_control.drone_arm_disarm:main",
            "hover_node = drone_control.hover:main",
            "pid_controller_node = drone_control.thrust_control_node:main",
            "line_following_node = drone_control.line_following:main",
            "line_make_node = drone_control.line_maker:main",
            "ch1_control_node = drone_control.challenge1_control:main",
            "takeoff_node= drone_control.takeoff:main",
            "new_hover_node = drone_control.hover_v2:main",
            "working_hover_node = drone_control.open_loop:main",
            "localization_node = drone_control.chal1_cntrl_final:main",
            "ch1_convertion_node = drone_control.challenge1_target_location:main",
            "obstacle_avoidance = drone_control.obstacle_avoidance:main",
            "controller_node = drone_control.controller_input:main",
            "new_open_loop = drone_control.open_loop_new:main",
            "move_to_point = drone_control.move_to_point:main",
            "spin_2_win = drone_control.spin_2_win:main",
            "ocean_motion = drone_control.ocean_motion:main"
        ],
    },
)
