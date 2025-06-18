from setuptools import find_packages, setup
import glob
import os

package_name = 'advantech_camera_ai'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install model files
        ('share/' + package_name + '/model', glob.glob('model/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='yakir@cogniteam.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'advantech_camera_ai_node = advantech_camera_ai.advantech_camera_ai_node:main'
        ],
    },
)