from setuptools import setup

package_name = 'my_auto'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dein_name',
    maintainer_email='deine@mail.de',
    description='Autonomes RC Auto mit Arduino und Sensoren (HC-SR04, IR)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wall_follower = my_auto.wall_follower:main',
            'arduino_controller = my_auto.arduino_controller:main',
        ],
    },
)
