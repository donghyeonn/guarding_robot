from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'qr_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('qr_package', 'launch', '*.launch.py'))),
    ],
    install_requires=['setuptools', 'opencv-python', 'numpy', 'mysql-connector-python'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='rokey@todo.todo',
    description='QR code processing package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qr_database = qr_package.qr_database:main',
            'qr_detector = qr_package.qr_detector:main',
            'qr_speak = qr_package.qr_speak:main',
        ],
    },
)
