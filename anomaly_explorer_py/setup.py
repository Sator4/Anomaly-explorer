from setuptools import find_packages, setup

package_name = 'anomaly_explorer_py'

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
    maintainer='sator',
    maintainer_email='sator@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'explore_action_node.py = anomaly_explorer_py.explore_action_node:main',
            'investigate_action_node.py = anomaly_explorer_py.investigate_action_node:main',
        ],
    },
)
