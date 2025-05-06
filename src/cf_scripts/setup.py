from setuptools import find_packages, setup

package_name = 'cf_scripts'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='darc',
    maintainer_email='darc@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "crazyflie_test=cf_scripts.crazyflie_test:main",
            "movement_test=cf_scripts.movement_test:main",
            "swarm_control=cf_scripts.swarm_control:main",
            "cf01_control=cf_scripts.cf01_control:main",
            "cf02_control=cf_scripts.cf02_control:main",
            "crazyflie_control=cf_scripts.crazyflie_control:main",
            "crazyflie_data_reader=cf_scripts.crazyflie_data_reader:main",
            "crazyflie_positioning=cf_scripts.crazyflie_positioning:main",
            "crazyflie_global_parameters=cf_scripts.crazyflie_global_parameters:main",
            "crazyflie_motor_test=cf_scripts.crazyflie_motor_test:main"
        ],
    },
)
