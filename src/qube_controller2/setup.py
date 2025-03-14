from setuptools import find_packages, setup

package_name = 'qube_controller2'

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
    maintainer='rune',
    maintainer_email='143501123+espenakk@users.noreply.github.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qube_controller2_node = qube_controller2.qube_controller2_node:main'
        ],
    },
)
