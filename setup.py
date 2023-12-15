import os
from glob import glob
from setuptools import setup

package_name = 'nebolab_gazebo_sim'
submodules = 'nebolab_gazebo_sim/control_lib'
scenarios = 'nebolab_gazebo_sim/scenarios_unicycle'
simulator = 'nebolab_gazebo_sim/simulator'


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, submodules, scenarios, simulator],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mwsatman',
    maintainer_email='8626150+mwsatman@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'expROS2_main = nebolab_gazebo_sim.expROS2_main:main',
            'CCTA_GoToGoal = nebolab_gazebo_sim.CCTA_GoToGoal:main',
            # 'cbf_single_integrator.py = nebolab_gazebo_sim.cbf_single_integrator:main',
            # 'go_to_goal.py = nebolab_gazebo_sim.go_to_goal:main',
        ],
    },
)
