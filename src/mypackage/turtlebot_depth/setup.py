
import glob
from setuptools import find_packages, setup

package_name = 'turtlebot_depth'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        ('share/' + package_name + '/launch',
            [f for f in glob.glob('launch/*.launch.py')]),
        # Install description files
        ('share/' + package_name + '/description',
            [f for f in glob.glob('description/*.urdf')]),
        # Install config files (including rviz config)
        ('share/' + package_name + '/config',
            [f for f in glob.glob('config/*')]),
        # Install models directory and README
        ('share/' + package_name + '/models',
            [f for f in glob.glob('models/*') if f.endswith(('.pth', '.md'))]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='dleon@csuchico.edu',
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
