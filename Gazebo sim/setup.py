from setuptools import find_packages, setup
from glob import glob

package_name = 'patrol_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='evenchova',
    maintainer_email='evenchova@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'multi_patrol = patrol_sim.multi_patrol:main',
		'patrol_bot = patrol_sim.patrol_bot:main',
        'nav_goal = patrol_sim.nav_goal:main',
        'set_initial_pose = patrol_sim.set_initial_pose:main',
        'multi_patrol_nav2 = patrol_sim.multi_patrol_nav2:main',
        ],
    },
)
