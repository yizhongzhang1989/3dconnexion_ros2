from setuptools import setup
from glob import glob
import os

package_name = 'spacemouse'


def _web_data_files():
    """Recursively collect all files under web/ for installation."""
    entries = []
    for dirpath, _, filenames in os.walk('web'):
        if not filenames:
            continue
        install_dir = os.path.join('share', package_name, dirpath)
        files = [os.path.join(dirpath, f) for f in filenames]
        entries.append((install_dir, files))
    return entries


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
    ] + _web_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yizhong Zhang',
    maintainer_email='yizhong@todo.todo',
    description='Web dashboard for 3Dconnexion SpaceMouse visualization',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dashboard_node = spacemouse.dashboard_node:main',
        ],
    },
)
