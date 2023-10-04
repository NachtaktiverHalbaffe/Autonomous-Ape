.. Sopias4 Framework documentation master file, created by
   sphinx-quickstart on Mon Jun 19 18:17:33 2023.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to Sopias4 Framework's documentation!
=============================================

.. toctree::
   :maxdepth: 3
   :caption: Setup

   installation.md
   troubleshooting.md
   advanced_infos.md

.. Heading is displayed via toctree
.. toctree::
   :maxdepth: 3
   :caption: Guides

   architecture.md
   things_to_consider.md
   plugin_bridges.md
   creating_gui.md
   running.md

API documentation
=================
This ROS2 package contains parts in Python and in C++. Although sharing the same ROS2 principles, they differ in their 
project structure and as a consequence in their documentation structure. For this reason the documentation is split into the docs 
for the Python and the C++ packages. The documentation provides the complete API documentation and integrated in there 
also additional guides and information if you have to follow some necessary steps e.g. correct configuration of some specific tool.

Overview
----------
The Python packages contains tools for GUI stuff, useful scripts for development and Navigation2 Python Plugins.
For this purpose, there are base node classes provided from which you can inherit to build your own implementation of that particular node.
Currently, there is a base node provided for building a PyQt5 GUI and base nodes for utilizing the Plugin bridges named PyPlugins so a Nav2 planner, costmap layers and
controller can be easy written in Python. After implementing a PyPlugin, make sure to configure the corresponding plugin bridge in the Nav2 stack. 
For this purpose look at the C++ documentation of the corresponding plugin bridge. The scripts, helping functions and other tools are provided in the tool package. In there, 
the tools are organized by their purpose or the subject for which they provide helping stuff.

The tools like the plugin bridges for Navigation 2 are implemented in C++. For easier implementation of the navigation plugins this C++ package contains plugin bridges for planner, costmap layers and controller.
This plugin bridges work out of the box without any need to code in C++. As a consequence, the focus and need for this documentation is the configuration part of the plugin bridges (although a complete API doc is provided).
If you want to implement your plugin in native C++, then you can follow the tutorials on the official Navigation 2 documentation: https://navigation.ros.org/plugin_tutorials/index.html.


.. Heading is displayed via toctree
.. toctree::
   :maxdepth: 3
   :caption: Interfaces

   message_def.md
   
.. Heading is displayed via toctree
.. toctree::
   :maxdepth: 3
   :caption: Python Documentation

   Nodes <sopias4_framework.nodes.rst>

.. toctree::
   :maxdepth: 5

   Tools <sopias4_framework.tools.rst>

.. Heading is displayed via toctree
.. toctree::
   :maxdepth: 3
   :caption: C++ Documentation

   ../generated/class_view_hierarchy.rst
   ../generated/file_view_hierarchy.rst 
   ../generated/unabridged_api.rst

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
