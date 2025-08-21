from setuptools import find_packages, setup
import os, glob

package_name = 'hana_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', '*.launch.xml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mkh',
    maintainer_email='kyung133851@pinkab.art',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hana_bringup = hana_bringup.hana_bringup:main',
            'camera = hana_bringup.camera:main',
            'hana_battery_publiser=hana_bringup.hana_battery_publiser:main',
            'hana_teleop_keyboard = hana_bringup.hana_teleop_keyboard:main'
        ],
    },
)
