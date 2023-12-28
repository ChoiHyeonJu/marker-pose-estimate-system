from setuptools import setup
import glob
import os
package_name = 'exp_logger'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
          glob.glob(os.path.join('launch', '*.launch.py'))),
        ('share/' + package_name + '/params',
          glob.glob(os.path.join('params', '*.yaml')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dh',
    maintainer_email='dh@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'exp_logger = exp_logger.exp_logger:main'
        ],
    },
)
