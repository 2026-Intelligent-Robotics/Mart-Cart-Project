from setuptools import find_packages, setup

package_name = 'cart_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='EWHA Dynamics',
    description='Human following and shelf docking system using LiDAR',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'human_following = cart_control.human_following:main',
            'shelf_docking = cart_control.shelf_docking:main',
            'object_detector = cart_control.object_detector:main',
        ],
    },
)
