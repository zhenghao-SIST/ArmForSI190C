from setuptools import find_packages, setup

package_name = 'coordination_publisher'

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
    maintainer='li',
    maintainer_email='lizhenghao@shanghaitech.edu.cn',
    description='coordination gui publisher',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui_node = coordination_publisher.gui_node:main'
        ],
    },
)
