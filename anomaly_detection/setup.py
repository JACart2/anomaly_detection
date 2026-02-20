from setuptools import setup

package_name = 'anomaly_detection'

setup(
    name=package_name,
    version='0.0.1',
    packages=['anomaly_detection'],
    install_requires=['setuptools', 'anomaly_msg'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Aggregates anomaly messages and writes to bag',
    license='Apache-2.0',
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            'anomaly_detection_node = anomaly_detection.anomaly_detection_node:main',
        ],
    },
)

