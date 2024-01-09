from setuptools import find_packages, setup
from os import path
from glob import glob

package_name = 'uol_tidybot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (path.join('share', package_name, 'launch'), glob(path.join('launch', '*launch.[pxy][yma]*'))),
        (path.join('share', package_name, 'param'), glob(path.join('param', '*.yaml'))),
        (path.join('share', package_name, 'worlds'), glob(path.join('worlds', '*.world'))),
        (path.join('share', package_name, 'meshes'), glob(path.join('meshes', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marc Hanheide',
    maintainer_email='marc@hanheide.net',
    description='CMP3103 teaching and assessment relevant package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)