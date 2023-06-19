"""
This scripts automatically generates the documentation
"""

import subprocess
import os


PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "../..", ""))

# Generatie Sphinx files (Python)
subprocess.call(["sphinx-apidoc", "-f", "-o", f"{PATH}/doc/source", f"{PATH}/sopias4_framework/"])

# Generate HTML File
subprocess.call(["rosdoc2", "build", "-p", f"{PATH}", "-o", f"{PATH}/doc/output", "-d", f"{PATH}/doc/build"])
