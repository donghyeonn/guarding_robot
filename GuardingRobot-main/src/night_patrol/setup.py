from setuptools import find_packages, setup

package_name = 'night_patrol'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/night_patrol.launch.py']),
        ('share/' + package_name, ['night_patrol/way2.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='zin1375@gmail.com',

    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'control_node = night_patrol.control_node_lifecycle:main',
        'patrol_nav2 = night_patrol.patrol_nav2_lifecycle:main',
        'patrol_nav3 = night_patrol.patrol_nav3_lifecycle:main',
        ],
    },
)
