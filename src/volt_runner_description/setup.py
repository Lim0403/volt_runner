from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'volt_runner_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # 1. Ament 리소스 마커 파일 (필수)
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        
        # 2. package.xml (필수)
        ('share/' + package_name, ['package.xml']),
        
        # 3. Launch 파일
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*.launch.py'))),
        
        # 4. Config 파일
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),

        # 5. RViz 파일
        (os.path.join('share', package_name, 'rviz'),
            glob(os.path.join('rviz', '*.rviz'))),

        # 6. Mesh 파일
        (os.path.join('share', package_name, 'meshes'),
            glob(os.path.join('meshes', '*.dae'))),

        # 7. URDF 파일 (모든 하위 폴더 포함)
        (os.path.join('share', package_name, 'urdf', 'controllers'),
            glob(os.path.join('urdf', 'controllers', '*.urdf.xacro'))),
        (os.path.join('share', package_name, 'urdf', 'mech'),
            glob(os.path.join('urdf', 'mech', '*.urdf.xacro'))),
        (os.path.join('share', package_name, 'urdf', 'properties'),
            glob(os.path.join('urdf', 'properties', '*.urdf.xacro'))),
        (os.path.join('share', package_name, 'urdf', 'robots'),
            glob(os.path.join('urdf', 'robots', '*.urdf.xacro'))),
        (os.path.join('share', package_name, 'urdf', 'sensors'),
            glob(os.path.join('urdf', 'sensors', '*.urdf.xacro'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lim',
    maintainer_email='lim@todo.todo',
    description='Description package for volt_runner',
    license='Apache-2.0',
    tests_require=['pytest'],
)