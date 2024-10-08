import os
from glob import glob
from setuptools import setup

package_name = 'robotik_projekt'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='me',
    maintainer_email='dont@write.me',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive_with_laserscanner = robotik_projekt.drive_with_laserscanner:main',
            'line_following = robotik_projekt.line_following:main',
            'line_following_dark_line = robotik_projekt.line_following_dark_line:main',
            'stoplight = robotik_projekt.stoplight:main',
            'state_machine = robotik_projekt.state_machine:main',
            'laserscanner_debug = robotik_projekt.laserscanner_debug:main',
        ],
    },
)
