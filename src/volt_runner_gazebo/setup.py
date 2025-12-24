from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'volt_runner_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch 파일 설치
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py')) +
            glob(os.path.join('launch', '*.py'))
        ),
        # 노드 스크립트 파일 설치
        (os.path.join('share', package_name, 'scripts'),
            glob(os.path.join('scripts', '*.py'))),
        # Worlds 파일 설치
        (os.path.join('share', package_name, 'worlds'), 
            glob(os.path.join('worlds', '*.sdf'))),
        # Config 파일 설치
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
        # Environment 파일 설치
        (os.path.join('share', package_name, 'environment'),
            glob(os.path.join('environment', '*.*'))),
            
        # --- [수정됨] Models 폴더 설치 ---
        (os.path.join('share', package_name, 'models', 'turtlebot3_world'),
            glob(os.path.join('models', 'turtlebot3_world', 'model.*'))),
        (os.path.join('share', package_name, 'models', 'turtlebot3_world', 'meshes'),
            glob(os.path.join('models', 'turtlebot3_world', 'meshes', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lim',
    maintainer_email='lim@todo.todo',
    description='Gazebo simulation package for volt_runner',
    license='Apache-2.0',
    tests_require=['pytest'],

    # # ✅ 여기만 새로 추가됨!!!
    # entry_points={
    #     'console_scripts': [
    #         'coil_pose_publisher = volt_runner_gazebo.coil_pose_publisher:main',
    #     ],
    # },
)
