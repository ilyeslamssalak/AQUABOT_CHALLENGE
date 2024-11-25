from setuptools import find_packages, setup

package_name = 'navigator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        'navigator': ['occupency_grid.npy'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kenobi',
    maintainer_email='matis.viozelange@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_planner = navigator.path_planner:main',
            'tf_manager = navigator.tf_manager:main',
            'occupancy_grid = navigator.occupancy_grid:main',
            'qr_code_detector = navigator.qr_code_detector:main',
            'critical_windturbine_position = navigator.critical_windturbine_position:main'
        ],
    },
)
