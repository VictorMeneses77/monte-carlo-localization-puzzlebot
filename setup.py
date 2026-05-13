import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'puzzlebot_localization'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jjj',
    maintainer_email='jjjau03@gmail.com',
    description='Monte Carlo Localization for Puzzlebot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'monte_carlo_localization = puzzlebot_localization.monte_carlo_localization:main',
            'fake_scan_publisher = puzzlebot_localization.fake_scan_publisher:main',
            'teleop_key_auto_stop = puzzlebot_localization.teleop_key_auto_stop:main',
        ],
    },
)
