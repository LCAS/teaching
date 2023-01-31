from setuptools import setup

package_name = 'cmp3103m_ros2_code_fragments'

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
    maintainer='kasm-user',
    maintainer_email='jcox289@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_square = cmp3103_scripts.move_square:main',
            'colour_mask = cmp3103_scripts.colour_mask:main',
            'colour_center = cmp3103_scripts.colour_center:main',
            'colour_mover = cmp3103_scripts.colour_mover:main',
            'opencv_bridge = cmp3103_scripts.opencv_bridge:main',
            'colour_contours = cmp3103_scripts.colour_contours:main'
        ],
    },
)
