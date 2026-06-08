from setuptools import find_packages, setup

package_name = 'volt_runner_isaac'

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
    maintainer='lim',
    maintainer_email='lim@todo.todo',
    description='Isaac bridge and test tools for volt_runner',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wheel_target_test_publisher = volt_runner_isaac.wheel_target_test_publisher:main',
            'adapter_cmd = volt_runner_isaac.adapter_cmd:main',
            'adapter_feedback = volt_runner_isaac.adapter_feedback:main',
            'sim_odom_publisher = volt_runner_isaac.sim_odom_publisher:main',
            'sim_ik_node = volt_runner_isaac.sim_ik_node:main',
            'sim_fk_node = volt_runner_isaac.sim_fk_node:main',
        ],
    },
)