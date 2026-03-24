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
            'camera_anomaly_node = tester.camera_anomaly_publisher:main',
            'lidar_test_node = tester.lidar_test_node:main',
            'eta_anomaly_node = tester.eta_anomaly_publisher:main',
            'offline_eta_anomaly_node = tester.offline_eta_anomaly_publisher:main',
        ],
    },
)
