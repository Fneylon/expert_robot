from setuptools import find_packages, setup

package_name = 'expert_robot'
submodules = 'expert_robot/submodules'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'matplotlib'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='FionaNeylon2022@u.northwestern.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'df_trajectories = expert_robot.df_trajectories:main',
            'record_trajectories = expert_robot.record_trajectories:main',
        ],
    },
)
