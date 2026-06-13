from setuptools import find_packages, setup

package_name = 'cube_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=['cube_detector'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='duccio',
    maintainer_email='duccio@todo.todo',
    description='Red cube detector (HSV + RGB-D point cloud) for Franka FR3 pick-and-place',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
    'console_scripts': [
        'detector_node = cube_detector.detector_node:main',
    ],
},
)