from setuptools import find_packages, setup

package_name = 'night_pkg'

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
    maintainer_email='ljhwan1997@naver.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_depth_checker = night_pkg.yolo_depth_checker:main',
            'cam_2_map_frame_tf = night_pkg.cam_2_map_frame_tf:main',
            'marking = night_pkg.marking:main',
            'obj_chase = night_pkg.obj_chase:main',
            'nav_to_pose = night_pkg.nav_to_pose:main',
            'pose_transform = night_pkg.pose_printer:main'
        ],
    },
)
