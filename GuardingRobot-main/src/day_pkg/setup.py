from setuptools import find_packages, setup

package_name = 'day_pkg'

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
    maintainer_email='zin1375@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'control_node = day_pkg.control_node:main',
        'qr_detector = day_pkg.qr_detector:main',
        'visit_order = day_pkg.visit_order:main',
        'cctv1 = day_pkg.cctv1:main',
        'cctv2 = day_pkg.cctv2:main',
        ],
    },
)
