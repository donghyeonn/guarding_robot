from setuptools import find_packages, setup

package_name = 'yolo_tracker_pkg'

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
    maintainer='rokey',
    maintainer_email='rokey@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'cc1 = yolo_tracker_pkg.cc1:main',
        'cc2 = yolo_tracker_pkg.cc2:main',
        'night_cctv1 = yolo_tracker_pkg.night_cctv1:main',
    ],
},

)
