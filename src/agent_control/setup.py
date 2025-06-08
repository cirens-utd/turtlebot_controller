from setuptools import find_packages, setup

package_name = 'agent_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy'],
    zip_safe=True,
    maintainer='stufmuff',
    maintainer_email='stufmuff@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "consensus_node = agent_control.consensus:main",
            "formation_node = agent_control.LF_formation:main",
            "followMe_node = agent_control.followMe:main",
            "multi_formation_node = agent_control.LF_multi_formation:main",
            "calibration_node = agent_control.calibration:main"
        ],
    },
)
