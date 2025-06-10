from setuptools import find_packages, setup

package_name = 'keyboard_input'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robin Karlsson',
    maintainer_email='robin.karlsson0@gmail.com',
    description='TROS 2 node for publishing text strings from keyboard input',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_publisher = ' + package_name + '.keyboard_input:main',
        ],
    },
)
