import os
from setuptools import find_packages, setup

package_name = 'simple_diff_drive_sim'

data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]


def collect_nested_files(*directories):
    collected = []
    for directory in directories:
        for root, _, filenames in os.walk(directory):
            files = [os.path.join(root, filename) for filename in filenames]
            if files:
                collected.append(
                    (os.path.join('share', package_name, root), files))
    return collected


data_files.extend(collect_nested_files('launch', 'urdf', 'worlds', 'models'))


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='name',
    maintainer_email='email@example.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'simple_diff_drive = simple_diff_drive_sim.simple_diff_drive:main',
            'patch_depot_collision = simple_diff_drive_sim.patch_depot_collision:main',
            'random_walk_obstacle = simple_diff_drive_sim.random_walk_obstacle:main',
            'navigation_test = simple_diff_drive_sim.navigation_test:main',
        ],
    },
)
