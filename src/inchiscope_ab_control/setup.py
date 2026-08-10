from setuptools import find_packages, setup

package_name = 'inchiscope_ab_control'

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
    description='Diameter-to-pressure mapping and safety clamping for the anchoring balloons.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ab_control_node = inchiscope_ab_control.ab_control_node:main',
        ],
    },
)
