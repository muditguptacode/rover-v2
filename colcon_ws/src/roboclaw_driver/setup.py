from setuptools import find_packages, setup

package_name = 'roboclaw_driver'

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
    maintainer='Mudit Gupta',
    maintainer_email='mail@muditg.com',
    description='Roboclaw ros2 driver',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = roboclaw_driver.publisher_member_function:main',
            'listener = roboclaw_driver.subscriber_member_function:main',
        ],
    },
)
