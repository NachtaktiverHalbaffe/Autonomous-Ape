# Sopias4 Framework <!-- omit in toc -->
This package extends the ROS2 framework with additional tools, definitions and classes specific for this Sopias4 project. 

## Table of Contents <!-- omit in toc -->
- [Overview](#overview)
- [Documentation and Guides](#documentation-and-guides)
- [License](#license)

# Overview
This package extends the ROS2 framework with additional tools, definitions and classes specific for this Sopias4 project. 

It provides all necessary parts for the plugin bridges for the Navigation2 package because it's plugins can originally be only written in C++, so we have to utilize ROS2 services and actions to enable Python usage for these plugins.

It also provides basic ROS2 Nodes which can be extended for the Sopias4 usecases. In specific, it provides a basic node for the gui which can be used to develop the Pyqt5 plugin. The other nodes are the corresponding Python nodes for the Navigation2 plugins in which the Python implementation of the plugins can be done.

It also provide various tools provided in a library manner which are loosely organized. Read the docs for a complete list and details.

# Documentation and Guides
The documentation and useful guides are provided in the documentation. There the usage of the different tools of the framework is explained and all necessary information can be found there. It is provided as an ZIP package on the Releases section on Github. It also contains the installation guide.

Otherwise it can be generated from source. For this purpose run the `generate_docs.py` script inside of `sopias4_framework.tools.scripts`. Make sure you are running inside the Dev container/WSL to have all necessary dependencies installed.
 
To open it, locate and open the `index.html` file.

# License
Copyright 2023 Institute of Industrial Automation and Software Engineering, University of Stuttgart

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

  http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.