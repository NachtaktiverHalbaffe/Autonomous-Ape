# Sopias4 Framework <!-- omit in toc -->
This package extends the ROS2 framework with additional tools, definitions and classes specific for this Sopias4 project. 

## Table of Contents <!-- omit in toc -->
- [Overview](#overview)
- [Usage](#usage)
  - [PyBridges](#pybridges)
  - [Nodes](#nodes)
  - [Messages](#messages)
  - [Lib](#lib)
- [Developing](#developing)
  - [Generating Documentation](#generating-documentation)
- [License](#license)

# Overview
It provides all important message, service and action definitions, so that the ROS2 interfaces are standardized.

It also provides all necessary parts for the Python bridges for the Navigation2 package because it's plugins can originally be only written in C++, so we have to utilize ROS2 services and actions to enable Python usage for these plugins.

It also provides basic ROS2 Nodes which can be extended for the Sopias4 usecases. In specific, it provides a basic node for the gui which can be used to develop the Pyqt5 plugin. The other nodes are the corresponding Python nodes for the Navigation2 plugins in which the Python implementation of the plugins can be done.

It also provide various tools provided in a library manner which are loosely organized. Read the docs for a complete list and details.

# Usage
## PyBridges
When using a specific Python Bridge, it has to be configured in the Navigation2 configuration:
<!-- TODO provide example configuration -->

It uses ROS2 Services and Actions for bridging. The Nodes package provides Python plugin nodes which already implemented these Actions and Services and you only need to override the plugin-specific implementation methods which you want to extend e.g. the path planning.

## Nodes
## Messages
## Lib

# Developing
## Generating Documentation
This framework uses automatic documentation generation using `rosdoc2`. Under the hood this runs Doxygen for C++ generation and Sphinx for Python generation. C++ generation should work out of the box.

For Python generation we have to run the generation ourself because rosdoc2 isn't configured right out of the box. For this purpo"se, navigate to the root directory of this package and run `sphinx-apidoc -o doc/source sopias4_framework/` .

After that, the HTML site containing the documentation can be generated. Just run `rosdoc2 build -p <path to package>` in the terminal and it should generate and locate the documentation in `docs_output`

# License
<!-- TODO add licensing -->