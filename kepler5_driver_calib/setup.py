from setuptools import find_packages, setup

package_name = 'kepler5_driver_calib'
# submodules = "kepler5_driver_calib/submodules"
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # packages=[package_name,submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kepler',
    maintainer_email='kepler@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibCam2Base3world_old =  kepler5_driver_calib.calibCam2Base3world_old:main',
            'calibCam2Base3world =  kepler5_driver_calib.calibCam2Base3world:main',
        ],
    },
)
