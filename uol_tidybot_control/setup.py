from setuptools import setup

package_name = 'uol_tidybot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Geesara',
    maintainer_email='ggeesara@gmail.com',
    description='Basic Diff drive robot control',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'control_strategy = uol_tidybot_control.robot_control_strategy:main',
        ],
    },
)
