from setuptools import find_packages, setup

package_name = 'inchiscope_reconstruction'

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
    description='Offline 3D reconstruction from a recorded rosbag using Aurora-tracked camera poses.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'extract_and_select = inchiscope_reconstruction.extract_and_select_cli:main',
            'sparse_sanity_check = inchiscope_reconstruction.sparse_sanity_check_cli:main',
        ],
    },
)
