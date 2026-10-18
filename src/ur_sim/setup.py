from setuptools import setup, find_packages

package_name = 'ur_sim'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    package_dir={
        package_name: 'ur_sim'
    },
    scripts=['scripts/human_mover.py'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config',
            ['config/ur3e_controllers_gripper.yaml',
             'config/initial_positions.yaml']),
        ('share/' + package_name + '/urdf',
            ['urdf/ur3e_spawn_gripper.urdf.xacro',
             'urdf/robotiq_gripper_macro.xacro']),
        ('share/' + package_name + '/launch',
            ['launch/ur3e_cartesian_control.launch.py']),
        ('share/' + package_name + '/worlds',
            ['worlds/pick_place_demo.world',
             'worlds/human_inter_demo.world']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='admin-dev',
    maintainer_email='ashokdev.ros2@gmail.com',
    description='UR3e with Robotiq gripper simulation and control',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'proximity_monitor = ur_sim.proximity_monitor:main',
            'move_robot = ur_sim.move_robot:main',
            'human_mover_standalone = human_mover_standalone:main',
            'human_zone_tester = human_zone_tester:main',
            'safety_controlled_robot = safety_controlled_robot:main',
        ],
    },
)
