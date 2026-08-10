from setuptools import find_packages, setup

package_name = 'inchiscope_control'

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
    description='Top-level state machine sequencing AB anchoring and PBA motion through the operating modes.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'inchiscope_control_node = inchiscope_control.inchiscope_control_node:main',
        ],
    },
)
