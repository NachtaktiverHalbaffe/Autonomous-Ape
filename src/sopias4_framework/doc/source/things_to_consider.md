# Things to consider when using this framework
Sopias4-Framework need some constraints to be met to operate correctly when all things are implemented. Following, guides for following this constraints are given.

## Creating the Sopias4-Application package
In order for the launch files provided by this framework to launch properly, the ROS2-package of Sopias4-Application has to be named correctly. It must be named `sopias_application`. It doesn't matter if you setup a Python or a C++/Python combined package, however you most likely only need to write Python nodes, so ROS2 Python package should be just fine. You can run the following command to automatically create the right package (make sure to be located inside the `src/` directory of the workspace):
```bash
ros2 pkg create --build-type ament_python <package_name>ros2 pkg create --build-type ament_python sopias4_application
```
Inside the package, make sure to include the `sopias4_framework` and `sopias4_msgs` package dependencies to avoid errors during the build. For this purpose, add the following lines to `package.xml` so it looks like the following:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>sopias4_application</name>
  <version>0.0.1</version>
  <description>TODO: Package description</description>
  <maintainer email="LeMonkay.VTIT@tutanota.com">NachtaktiverHalbaffe</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>
  <!-- Python dependencies -->
  <depend>rclpy</depend>
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <!-- Ros2 packages -->
  <depend>sopias4_msgs</depend>
  <depend>sopias4_framework</depend>
  <depend>teb_local_planner</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```
## Creating the GUI
If you want to use the `sopias4-application` shortcut inside your terminal instead of `ros2 run sopias4_application <your gui node>`, then it is important that the gui class is named `GUI`, the main method which starts the GUI is named `main()` and that the file is named and exported as `gui.py`

## Configuring Navigation2 stack
Navigation 2 is configured via a YAML-file. Sopias4-Framework provides launchfiles for launching the stack and loading the configuration. For this purpose, the YAML configuration file must be named right and located in the right location like the following:
- Name: It must be named `nav2.yaml`. You can also have other configurations with other names, but these aren't loaded automatically out of the box
- Location: It must be located in `config` inside your Sopias4-Application package. Make sure to export these files into the shared resources when you create this directory. For this, add this to your build configuration:
  -  Python package (`ament_python`): Insert inside `setup.py`
        ```python
        import os
        from glob import glob

        from setuptools import find_packages, setup

        package_name = "sopias4_application"

        setup(
            name=package_name,
            version="0.0.1",
            packages=find_packages(exclude=["test"]),
            data_files=[
                ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
                ("share/" + package_name, ["package.xml"]),
                (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
                (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
            ],
            # Other stuff after that can be left at it is
        )
        ```
  - Python & C++ combined package (`ament_cmake`): Insert inside `CMakeLists.txt`:
     ```Cmake
     install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
     ```