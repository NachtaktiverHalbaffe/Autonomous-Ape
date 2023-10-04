"""
This scripts automatically generates the documentation. This script can be run without \
arguments from the console if being used for the Sopias4 Framework package because it's 
optimized for usage with this specific package.

However, it can be used outside the Sopias4 Framework Package, but in this case you have to make a 
few adjustments to your ROS2 package:
    - Create a folder `doc/` in your package and cd into it in the terminal
    - Run `sphinx-quickstart` and follow the steps. Make sure to select the option to separate source and build directories
    - Copy the content of the `conf.py` in the Sopias4 Framework, located under `doc/build`: \
            You can keep the meta-data specific to the package. If during the building an error occurs that\
            some specific module was not found, then place them in `MOCK_MODULES` inside of `conf.py`
    - Include the modules-rst file in your `index.rst`. Take `doc/source/index.rst` inside Sopias4 Framework as a reference

Then you can use generate_docs to generate the docs, but you have to make following adjustments:
    - Pass the path to the root of the ROS2-package as an argument
    - You can achieve this also by running it over the terminal with following command (passing an argument):\
        `python3 generate_docs.py -p  <path to ROS2-package>`
"""

import getopt
import os
import subprocess
import sys


def generate_docs(pkg_path: str | None = None):
    """
    Automatically generate the documentation. The default values are set for running in the Sopias4 Framework package,
    however you can pass a pkg_path to run it for your own package

    Args:
        pkg_path (str, optional): The path to the root of your ROS2-Pkg. If using default, it will use the path to the Sopias4 Framework package
    """
    if pkg_path is None:
        PKG_PATH = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "../../..", "")
        )
        # Generate Sphinx files (Python)
        subprocess.call(
            [
                "sphinx-apidoc",
                "-f",
                "-o",
                f"{PKG_PATH}/doc/source",
                f"{PKG_PATH}/sopias4_framework/",
            ]
        )
        # Run rosdoc2
        subprocess.call(
            [
                "rosdoc2",
                "build",
                "-p",
                f"{PKG_PATH}",
                "-o",
                f"{PKG_PATH}/doc/output",
                "-d",
                f"{PKG_PATH}/doc/build",
                "-c",
                f"{PKG_PATH}/doc/cross_reference",
            ]
        )
    else:
        # Generate Sphinx files (Python)
        subprocess.call(
            ["sphinx-apidoc", "-f", "-o", f"{pkg_path}/doc/source", f"{pkg_path}"]
        )
        # Run rosdoc2
        subprocess.call(
            [
                "rosdoc2",
                "build",
                "-p",
                f"{pkg_path}",
                "-o",
                f"{pkg_path}/doc/output",
                "-d",
                f"{pkg_path}/doc/build",
                "-c",
                f"{pkg_path}/doc/cross_reference",
            ]
        )


if __name__ == "__main__":
    # Use rosdoc2 if specified as a cli argument, otherwise use plain sphinx
    try:
        opts, args = getopt.getopt(sys.argv[1:], "p:", ["ros2-pkg-path"])
    except getopt.GetoptError:
        print("python3 generate_docs.py -p  <path to ROS2-package>")
        sys.exit(2)
    for opt, arg in opts:
        if opt in ("-p", "--ros2-pkg-path"):
            generate_docs(arg)
            sys.exit()

    generate_docs()
    sys.exit()
