import os
from glob import glob
from setuptools import setup

package_name = "nebolab_gazebo_sim"
control_lib = "nebolab_gazebo_sim/control_lib"
scenarios = "nebolab_gazebo_sim/scenarios_unicycle"
simulator = "nebolab_gazebo_sim/simulator"
data_files = [
    ("share/" + package_name, ["package.xml"]),
    ('share/ament_index/resource_index/packages',
    ['resource/' + package_name])
]


def package_files(data_files, directory_list):
    paths_dict = {}
    for directory in directory_list:
        for path, directories, filenames in os.walk(directory):
            for filename in filenames:
                file_path = os.path.join(path, filename)
                install_path = os.path.join("share", package_name, path)

                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)

                else:
                    paths_dict[install_path] = [file_path]

    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))

    return data_files


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name, simulator],
    data_files=package_files(
        data_files,
        ["launch/", "worlds/"],
    ),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mwsatman",
    maintainer_email="8626150+mwsatman@users.noreply.github.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "CCTA_GoToGoal = nebolab_gazebo_sim.CCTA_GoToGoal:main",
            "vicon_localization = nebolab_gazebo_sim.vicon_localization:main",
        ],
    },
)
