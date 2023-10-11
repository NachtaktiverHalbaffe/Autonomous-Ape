# Installation
This splits up into setting up a ROS2 workspace for the Sopias4 project and installing ROS2. It's recommended to first setup the workspace and then setting up ROS2, because the workspace contains necessary configuration files if you want to use the Dev Containers and Docker for easy setup. 

## Setting up ROS2 workspace
Clone this repository. When using Linux, it is recommended to clone it into the home directory of your linux user so it matches with the commands in this guide.
```Bash
# If git isn't installed
sudo apt update && apt install git

cd /home/<your linux user>
git clone https://github.tik.uni-stuttgart.de/IAS/sopias4_ws # Make sure the URL is right, can vary 
```
After you have installed ROS2, you can follow the installation of the development container.
## Installing ROS2 and necessary applications 
### Recommended: Running in Docker using Dev Containers (Visual Studio Code)
Using Visual Studio Code and Docker Containers will enable you to run your favorite ROS 2 Distribution without the necessity to change your operating system or use a virtual machine. This workspace provides a nearly finished configuration, but if you want to replicate it by your own (with a little different configuration) a tutorial can be found here: [https://docs.ros.org/en/iron/How-To-Guides/Setup-ROS-2-with-VSCode-and-Docker-Container.html](https://docs.ros.org/en/iron/How-To-Guides/Setup-ROS-2-with-VSCode-and-Docker-Container.html)

If you want to use the pre-configured Dev Container, then the configuration is found in `.devcontainer/`. This also installs all required packages automatically on setup. However, a few steps are still needed:
1. Install `docker`   
    - For Ubuntu:
        ```Bash
        sudo apt-get update
        sudo apt-get install ca-certificates curl gnupg

        sudo install -m 0755 -d /etc/apt/keyrings
        curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
        sudo chmod a+r /etc/apt/keyrings/docker.gpg

        echo \
        "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
        "$(. /etc/os-release && echo "$VERSION_CODENAME")" stable" | \
        sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

        sudo apt-get update
        sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
        ```
        You can verify your install by running
        ```bash
        sudo docker run hello-world
        ```
    - For Windows
      - Open powershell as administrator
      - Install WSL2 
        - Open Powershell (or Terminal) as administrator
        - Run command `wsl --install`. This installs a Ubuntu-Subsystem
        - In pure Windows fashion, restart your computer
        - After the restart, a terminal should open a window and complete installation. You should be prompted to enter credentials for the new user. It doesnt matter, but make sure you can remember them as you can need them in the future
      - Install drivers for graphical applications:
          - Download driver from [https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps). Note that these could be already installed. In this case you can skip the installation
      - Install Docker
        - Download from [https://docs.docker.com/desktop/install/windows-install/](https://docs.docker.com/desktop/install/windows-install/)
        - Execute
      - In pure Windows fashion, restart your computer
    <!-- - TODO Add windows stuff -->
2. Only Linux: Add your current linux user which is used inside the container to the docker group:
    ```bash
    sudo groupadd docker
    sudo usermod -aG docker $USER
    newgrp docker
    ```
3. Only Windows: Make sure Docker (inluding WSL2) is running
4. Open the workspace
5. Download and install Visual Studio Code
6. In Visual Studio Code: Go to the extensions tab and install the extension Dev Containers and WSL(Windows only)
7. Set the `ROS_DOMAIN_ID` inside `devcontainer.json` to the number of the Turtlebot you want to control e.g. set it to 2 if you want to use /turtle2. Use the devcontainer-file which sits in the folder which represents your GPU (note: Nvidia is untested, remember that often the iGPU of the processor is configured to be used in Linux)
8. Optional: Replace `ros2` inside `devcontainer.json` with your preferred username under which the container runs
9.  Open Command Palette (either use `View->Command Palette...` or `Ctrl+Shift+P`), then search and/or run command `Dev Containers: (Re-)build and Reopen in Container`. Take the one which suits best for your platform 
10. Lean back and take a coffee. The initial start of the environment can take a while (5-10 minutes)

### Run directly on host (deprecated)
Follow [https://docs.ros.org/en/iron/Installation.html](https://docs.ros.org/en/iron/Installation.html) and use the humble release. Remind that the guides in this repository where written for Linux (Ubuntu), so if you are using another OS you may need to adapt a few commands. Additionally, all tools are tailored to the dev containers and thus some rare things aren't supported on bare metal out of the box. Finally, all other guides are assuming you are using the dev containers, so you may have to modify some stuff by yourself if following the guides of this documentation.

In general you can setup your environment by "running" the `DOCKERFILE`  after line 16 by hand:
- Execute every `RUN` command in your terminal
- Set the environment variables which are set with the `ENV` command. Simply enter `export <variable extracted from dockerfile>` in your terminal. Note: you have to do this every time you open a new terminal. To make this permanent, then use the command `echo "export <variable extracted from dockerfile>" >>/home/<your linux user>/.bashrc`
- Replace all variables like the following:
  - Replace `${ROS-DISTRO}` with `humble`
  - Replace `${WORKSPACE}` with the (absolute) path to your workspace
  - Replace `${USERNAME}` with your linux username

 You have to source ROS2 and the workspace every time you open a terminal and your IDE needs to be started from a sourced terminal so you get all linter and syntax highlighting capabilities. For this you have to run these commands in the opened terminal:
```bash
source /opt/ros/humble/setup.bash
source source <path to ros workspace>/install/setup.bash
```
If you want this done automatically each time, when run this commands once:
```bash
echo "source /opt/ros/humble/setup.bash" >>/home/<your linux user>/.bashrc
echo "if [ -f <path to ros workspace>/install/setup.bash ]; then source <path to ros workspace>/install/setup.bash; fi" >> /home/<your linux user>/.bashrc
```

