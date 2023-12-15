from setuptools import find_packages, setup

package_name = 'ccta2024_scalable'
submodules = 'ccta2024_scalable/control_lib'
scenarios = 'ccta2024_scalable/scenarios_unicycle'
simulator = 'ccta2024_scalable/simulator'


setup(
    name=package_name,
    version='0.0.0',
    # packages=find_packages(exclude=['test']),
    packages=[package_name, submodules, scenarios, simulator],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='localadmin',
    maintainer_email='8626150+mwsatman@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'expROS2_main = ccta2024_scalable.expROS2_main:main',
            'CCTA_GoToGoal = ccta2024_scalable.CCTA_GoToGoal:main',
            # 'cbf_single_integrator.py = ccta2024_scalablecbf_single_integrator:main',
            # 'go_to_goal.py = ccta2024_scalable.go_to_goal:main',
        ],
    },
)
