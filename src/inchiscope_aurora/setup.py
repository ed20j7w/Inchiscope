from setuptools import find_packages, setup

package_name = 'inchiscope_aurora'

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
    description='NDI Aurora EM tracker interface: publishes tip pose and broadcasts the tf2 transform.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aurora_tracker_node = inchiscope_aurora.aurora_tracker_node:main',
        ],
    },
)
