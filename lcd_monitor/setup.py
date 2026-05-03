from setuptools import setup

package_name = 'lcd_monitor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mike',
    maintainer_email='mike@todo.todo',
    description='The lcd_monitor package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sm_lcd_node = lcd_monitor.sm_lcd_driver:main',
        ],
    },
)
