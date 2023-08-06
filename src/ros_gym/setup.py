from setuptools import setup
import os 
from glob import glob

package_name = 'ros_gym'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tsl',
    maintainer_email='fahmy.adan14@imperial.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reset_service= ros_gym.reset_service_node:main',
            'reset_client= ros_gym.reset_client_node:main',
        ],
    },
)
