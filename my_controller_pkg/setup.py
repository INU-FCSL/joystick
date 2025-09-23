from setuptools import find_packages, setup

package_name = 'my_controller_pkg'

setup(
    name='my_controller_pkg',
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jwpark',
    maintainer_email='jwpark@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'xbox_controller = my_controller_pkg.xbox_full_control:main',
            'direct_arm_control = my_controller_pkg.direct_arm_control:main',
        ],
    },
)
