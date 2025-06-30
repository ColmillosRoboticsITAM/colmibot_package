from setuptools import find_packages, setup

package_name = 'colmibot'

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
    maintainer='ColmillosRoboticsITAM',
    maintainer_email='colmillos.robotica@gmail.com',
    description='Colmibot controller with keyboard and joystick',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keyboard = colmibot.teleop_keyboard:main',
            'teleop_joystick = colmibot.teleop_joystick:main'
        ],
    },
)
