from setuptools import find_packages
from setuptools import setup

package_name = 'imp_ros_network_stats'

setup(
    name=package_name,
    version='0.9.4',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Khaleel Husain',
    author_email='khaleel@imperiumdrive.com',
    maintainer='Khaleel Husain',
    maintainer_email='khaleel@imperiumdrive.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'Python nodes which were previously in the ros2/examples repository '
        'but are now just used for demo purposes.'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros_network_stats_listener_exe = imp_ros_network_stats.src.listener:main',
            'ros_network_stats_talker_exe = imp_ros_network_stats.src.talker:main',
        ],
    },
)
