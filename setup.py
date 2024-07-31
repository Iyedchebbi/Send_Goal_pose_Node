from setuptools import setup

package_name = 'navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Package for robot navigation in ROS 2',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigate_to_room = navigation.navigate_to_room:main',
        ],
    },
)

