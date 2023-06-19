# Sopias4 Framework
# Overview
# Running
# Developing
## Generating Documentation
This framework uses automatic documentation generation using `rosdoc2`. Under the hood this runs Doxygen for C++ generation and Sphinx for Python generation. C++ generation should work out of the box.

For Python generation we have to run the generation ourself because rosdoc2 isn't configured right out of the box. For this purpo"se, navigate to the root directory of this package and run `sphinx-apidoc -o doc/source sopias4_framework/` .

After that, the HTML site containing the documentation can be generated. Just run `rosdoc2 build -p <path to package>` in the terminal and it should generate and locate the documentation in `docs_output`