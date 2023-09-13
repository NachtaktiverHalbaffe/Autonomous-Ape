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
9. Change the NTP configuration of the Turtlebot itself:
   1. SSH into the Turtlebot
   2. Edit `/etc/chrony/chrony.conf` e.g. `sudo nano /etc/chrony/chrony.conf`
   3. Change the content so it does look like this:
      ```bash
      # Welcome to the chrony configuration file. See chrony.conf(5) for more
      # information about usuable directives.

      server rustime01.rus.uni-stuttgart.de iburst
      server rustime02.rus.uni-stuttgart.de iburst
      server 192.168.178.28 iburst # This should be the IP adress of the host where Sopias4 Fleetbroker runs on
      # Enable serving time to ntp clients on 192.168.186.0 subnet.
      allow 192.168.186.0/24

      # Allow local sync
      local stratum 10

      # This directive specify the location of the file containing ID/key pairs for
      # NTP authentication.
      keyfile /etc/chrony/chrony.keys

      # This directive specify the file into which chronyd will store the rate
      # information.
      driftfile /var/lib/chrony/chrony.drift

      # Uncomment the following line to turn logging on.
      #log tracking measurements statistics

      # Log files location.
      logdir /var/log/chrony

      # Stop bad estimates upsetting machine clock.
      maxupdateskew 100.0

      # This directive enables kernel synchronisation (every 11 minutes) of the
      # real-time clock. Note that it can’t be used along with the 'rtcfile' directive.
      rtcsync

      # Step the system clock instead of slewing it if the adjustment is larger than
      # one second, but only in the first three clock updates.
      makestep 1 3
      ```

## Time synchronization
It is important for the Turtlebots and all the clients that the time is synchronized. Otherwise you could run into problems. To achive this NTP is utilized. All the clients (especially the Turltebot) synchronizes with the host which runs the Sopias4-Fleetbroker. If the development container is used and the Turtlebots are setup like instructed then everything should be setup already. However, if you run into timesycning issues, then it is useful to check if the NTP configuration is still working and if chrony (used as NTP server and clients) is running fine.