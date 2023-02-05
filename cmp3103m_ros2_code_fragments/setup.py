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
            'chat_receiver = cmp3103m_ros2_code_fragments.chat_receiver:main',
            'chat_sender = cmp3103m_ros2_code_fragments.chat_sender:main',
            'roamer = cmp3103m_ros2_code_fragments.roamer:main',
            'move_square = cmp3103m_ros2_code_fragments.move_square:main',
            'colour_mask = cmp3103m_ros2_code_fragments.colour_mask:main',
            'colour_mask2 = cmp3103m_ros2_code_fragments.colour_mask2:main',
            'colour_center = cmp3103m_ros2_code_fragments.colour_center:main',
            'colour_mover = cmp3103m_ros2_code_fragments.colour_mover:main',
            'opencv_bridge = cmp3103m_ros2_code_fragments.opencv_bridge:main',
            'colour_contours = cmp3103m_ros2_code_fragments.colour_contours:main'
        ],
    },
)
