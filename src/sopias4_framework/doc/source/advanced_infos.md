# Advanced information (mainly for supervisors)
This information should't be needed to develop Sopias4-Application. These sections provide additional informations, hints and guides if some errors occur which arent covered in the troubleshooting guide or if some central systems or parts must be reinstalled or completly setup from scratch.

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