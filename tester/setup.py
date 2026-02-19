from setuptools import setup

package_name = 'tester'

setup(
    name=package_name,
    version='0.0.1',
    packages=['tester'],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Test nodes to publish fake anomaly messages',
    license='Apache-2.0',
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            'fake_camera_node = tester.fake_camera_node:main',
            'lidar_test_node = tester.lidar_test_node:main',
        ],
    },
)
