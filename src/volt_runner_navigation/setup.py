from setuptools import setup
import os
from glob import glob

package_name = 'volt_runner_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],  # 지금은 순수 launch/config 패키지라 비워둬도 됨
    data_files=[
        # ament_index 등록
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        # package.xml
        ('share/' + package_name, ['package.xml']),

        # launch 파일 전체 (.py)
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.py'))),

        # config
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*'))),

        # maps
        (os.path.join('share', package_name, 'maps'),
         glob(os.path.join('maps', '*.*'))),

        # rviz
        (os.path.join('share', package_name, 'rviz'),
         glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lim',
    maintainer_email='lim@todo.todo',
    description='Navigation (SLAM, Nav2) package for the volt_runner',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={},
)
