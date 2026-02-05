from setuptools import find_packages, setup

package_name = 'reference_input_node'

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
    maintainer='armancv',
    maintainer_email='armancv@stud.ntnu.no',
    description='Python Client Server',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reference_input_node = reference_input_node.service_member_function:main',
        ],
    },
)
