"""
This package provides all important message, service and action definitions, so that the ROS2 interfaces are standardized.

It also provides all necessary parts for the Python bridges for the Navigation2 package because it's plugins can originally
be only written in C++, so we have to utilize ROS2 services and actions to enable Python usage for these plugins.

It also provides basic ROS2 Nodes which can be extended for the Sopias4 usecases. In specific, it provides a basic node 
for the gui which can be used to develop the Pyqt5 plugin. The other nodes are the corresponding Python nodes for the 
Navigation2 plugins in which the Python implementation of the plugins can be done.

It also provide various tools provided in a library manner which are loosely organized. Read the docs for a complete list 
and details.
"""
