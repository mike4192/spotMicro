from setuptools import find_packages, setup

package_name = 'spot_micro_mujoco_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mujoco_sim_launch.py']),
        ('share/' + package_name + '/models', ['models/spot_micro_sim.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='MuJoCo simulation bridge for SpotMicro robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mujoco_sim_node = spot_micro_mujoco_sim.mujoco_sim_node:main',
        ],
    },
)
