from setuptools import find_packages, setup

package_name = 'lidar_error_checker'

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
    maintainer='vscode',
    maintainer_email='vscode@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [        # Format: '<name of node> = <package>.<name of file>:main'
            'lidar_breadth_search = lidar_error_checker.lidar_breadth_search:main',
            'lidar_depth_search= lidar_error_checker.lidar_depth_search:main',
        ],
    },
)
