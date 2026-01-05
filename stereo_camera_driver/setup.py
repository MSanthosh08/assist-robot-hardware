from setuptools import setup

package_name = 'stereo_camera_driver'

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
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='Stereo CSI camera driver',
    license='Apache License 2.0',

    entry_points={
        'console_scripts': [
            'stereo_camera_node = stereo_camera_driver.stereo_camera_node:main',
        ],
    },
)
