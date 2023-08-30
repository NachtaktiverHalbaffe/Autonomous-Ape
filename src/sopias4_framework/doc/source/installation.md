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
After you have installed ROS2, you can run an initial build of this workspace:
```Bash
cd <path to your workspace>
colcon build
```

## Installing ROS2 and necesary applications 
### Recommended: Running in Docker using Dev Containers (Visual Studio Code)
Using Visual Studio Code and Docker Containers will enable you to run your favorite ROS 2 Distribution without the necessity to change your operating system or use a virtual machine. This workspace provides a nearly finished configuration, but if you want to replicate it by your own (with a little different configuration) a tutorial can be found here: https://docs.ros.org/en/iron/How-To-Guides/Setup-ROS-2-with-VSCode-and-Docker-Container.html

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
    - For other Linux distros or OS search for the corresponding guides
2. Add your current linux user which is used inside the container to the docker group:
    ```bash
    sudo groupadd docker
    sudo usermod -aG docker $USER
    newgrp docker
    ```
3. Rename the right devcontainer file inside `.devcontainer/` to `devcontainer.json` depending on your graphics card. Note: Only `devcontainer_intel.json` is tested.
4. Optional: Replace `nachtaktiverhalbaffe` inside `.devcontainer/devcontainer.json` with your preferred username under which the container runs
5. Open Command Palette (either use `View->Command Palette...` or `Ctrl+Shift+P`), then search and/or run command `Dev Containers: (Re-)build and Reopen in Container`
6. Lean back and take a coffee. The initial start of the environment can take a while (5-10 minutes)

### Run directly on host
Follow [https://docs.ros.org/en/iron/Installation.html](https://docs.ros.org/en/iron/Installation.html) and use the humble release. Remind that the guides in this repository where written for Linux (Ubuntu), so if you are using another OS you may need to adapt a few commands.

In general you can setup your environment by "running" the `DOCKERFILE`  after line 16 by hand:
- Execute every `RUN` command in your terminal
- Set the environment variables which are set with the `ENV` command. Simply enter `export <variable extracted from dockerfile>` in your terminal. Note: you have to do this every time you open a new terminal. To make this permanent, then use the command `echo "export <variable extracted from dockerfile>" >>/home/<your linux user>/.bashrc`
- Replace all variables like the following:
  - Replace `${ROS-DISTRO}` with `humble`
  - Replace `${WORKSPACE}` with the (absolute) path to your workspace
  - Replace `${USERNAME}` with your linux username

 You have to source ROS2 and the workspace everytime you open a terminal and your IDE needs to be started from a sourced terminal so you get all linter and syntax highlighting capabilities. For this you have to run these commands in the opened terminal:
```bash
source /opt/ros/humble/setup.bash
source source <path to ros workspace>/install/setup.bash
```
If you want this done automatically each time, when run this commands once:
```bash
echo "source /opt/ros/humble/setup.bash" >>/home/<your linux user>/.bashrc
echo "if [ -f <path to ros workspace>/install/setup.bash ]; then source <path to ros workspace>/install/setup.bash; fi" >> /home/<your linux user>/.bashrc
```

## Setting up Turtlebots
1. Download the latest ISO at [http://download.ros.org/downloads/turtlebot4/](http://download.ros.org/downloads/turtlebot4/)
2. Install a disk flashing application e.g. [Balena Etcher](https://etcher.balena.io)
3. Remove the SD-Cart from the Raspberry Pi in the Turtlebot:
   1. Screw of the top plate and the long standoffs
   2. Lift of the plate where the LIDAR is mounted on
   3. The SD card is on the bottom side of the Raspberry Pi. Removing it is a little bit tricky
4. Connect the SD cart with your PC and use your flashing programm to flash the downloaded ISO onto the SD cart
5. Follow the setup instruction: [https://turtlebot.github.io/turtlebot4-user-manual/setup/basic.html#robot](https://turtlebot.github.io/turtlebot4-user-manual/setup/basic.html#robot)
6. Connect the Create3 to the access point: [https://turtlebot.github.io/turtlebot4-user-manual/setup/simple_discovery.html#create-3](https://turtlebot.github.io/turtlebot4-user-manual/setup/simple_discovery.html#create-3)
7. Update the firmware of the Create 3 over the webserver. If connected to Wifi, it will automatically download the latest image
8. Change the NTP Configuration of the Create 3: [https://github.com/turtlebot/turtlebot4/issues/216#issue-1797043215](https://github.com/turtlebot/turtlebot4/issues/216#issue-1797043215)