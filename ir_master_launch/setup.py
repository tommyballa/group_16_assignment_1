from setuptools import find_packages, setup

package_name = 'ir_master_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/ir_master_launch']),
        ('share/ir_master_launch', ['package.xml']),
        ('share/ir_master_launch/launch', [
            'launch/general.launch.py',
            'launch/assignment_1.launch.py'
            ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='paolo',
    maintainer_email='paolo@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
