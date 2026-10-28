from glob import glob

from setuptools import setup

package_name = 'stretch4_mission'

setup(
    name=package_name,
    version='0.4.0',
    packages=[package_name],
    package_data={package_name: ['web/*']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'check_robot = stretch4_mission.check_robot:main',
            'sync_policies = stretch4_mission.sync_policies:main',
            'mission_controller = stretch4_mission.mission_controller:main',
            'mission_ui = stretch4_mission.mission_ui:main',
            'policy_bridge = stretch4_mission.policy_bridge:main',
        ],
    },
)
