from setuptools import find_packages, setup

package_name = 'servos_node'

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
    maintainer='javideus',
    maintainer_email='javi.rm2005@gmail.com',
    description='This is the package to control the servo motors',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'server = ' + package_name + '.servo_controller_server:main',
		'client = ' + package_name + '.servo_controller_client:main',
        ],
    },
)
