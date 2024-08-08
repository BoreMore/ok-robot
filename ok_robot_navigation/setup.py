from setuptools import setup

package_name = 'ok_robot_navigation'

setup(
    name=package_name,
    version='0.0.0',
    #package_dir={'': 'src'},
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    url='',
    maintainer='orrin',
    maintainer_email='orrin.dahanaggamaarachchi@mail.utoronto.com',
    description='Learning systems Lab Stretch Laptop Development',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_subscriber = ok_robot_navigation.odom_subscriber:main'
        ],
    },
)
