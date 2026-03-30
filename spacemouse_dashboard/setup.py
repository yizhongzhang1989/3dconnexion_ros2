from setuptools import setup
from glob import glob
import os

package_name = 'spacemouse_dashboard'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'web'),
            glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yizhong Zhang',
    maintainer_email='yizhong@todo.todo',
    description='Web dashboard for 3Dconnexion SpaceMouse visualization',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dashboard_node = spacemouse_dashboard.dashboard_node:main',
        ],
    },
)
