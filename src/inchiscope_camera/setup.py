from setuptools import find_packages, setup

package_name = 'inchiscope_camera'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Inchiscope',
    maintainer_email='ed20j7w@leeds.ac.uk',
    description='Publishes the NanEye tip camera feed, grabbed off a USB capture card as a standard V4L2/UVC device.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = inchiscope_camera.camera_node:main',
            'camera_viewer_node = inchiscope_camera.camera_viewer_node:main',
        ],
    },
)
