# Advanced information (mainly for supervisors)
This information should't be needed to develop Sopias4-Application. These sections provide additional information, hints and guides if some errors occur which aren't covered in the troubleshooting guide or if some central systems or parts must be reinstalled or completly setup from scratch.

## Setting up Turtlebot's
1. Download the latest ISO at [http://download.ros.org/downloads/turtlebot4/](http://download.ros.org/downloads/turtlebot4/)
2. Install a disk flashing application e.g. [Balena Etcher](https://etcher.balena.io)
3. Remove the SD-Cart from the Raspberry Pi in the Turtlebot:
   1. Screw of the top plate and the long standoffs
   2. Lift of the plate where the LIDAR is mounted on
   3. The SD card is on the bottom side of the Raspberry Pi. Removing it is a little bit tricky
4. Connect the SD cart with your PC and use your flashing application to flash the downloaded ISO onto the SD cart
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
      # information about usable directives.

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

      # This directive enables kernel synchronization (every 11 minutes) of the
      # real-time clock. Note that it can’t be used along with the 'rtcfile' directive.
      rtcsync

      # Step the system clock instead of slewing it if the adjustment is larger than
      # one second, but only in the first three clock updates.
      makestep 1 3
      ```
## Workarounds for Turtlebot4 to improve performance
Like mentioned in the troubleshooting-guide, the Turtlebot has some stability issues sometimes leading to a crash which is indicated by a red light on the power button. However there are workarounds which can be applied to improve this situation a providing a more stable experience. In the future these can become obsolete due to updates from Turtlebot4 or the Create3 base, so check if these are still needed.

### Disable Turtlebot4 Diagnostics
The Turtlebot4 has some nodes running for diagnostic purpose. These can be disabled to reduce the CPU load on the Create3 base significantly. For this, do following:
1. SSH into the Turtlebot: `ssh ubuntu@<Ip address of raspberry pi>` (default password is turtlebot4)
2. Run `turtlebot4-setup`
3. Go under ROS Setup -> Bash Setup -> Select TURTLEBOT4_DIAGNOSTICS -> Disabled -> Save
4. Back out to the main menu and apply settings
5. The nodes should restart and diagnostic node should disappear

### Apply custom RMW profiles
The ROS Middleware settings can be customized both on the Turtlebot4 and the Create3.

For the Create3 follow following guide: [https://iroboteducation.github.io/create3_docs/webserver/rmw-profile-override/](https://iroboteducation.github.io/create3_docs/webserver/rmw-profile-override/). Use the following profile  (replace XXX.XXX.XXX.XXX with IP-address of Create3 base):
```bash
<?xml version="1.0" encoding="UTF-8"?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <profiles>
        <!-- UDPv4 Transport profile -->
        <transport_descriptors>
            <transport_descriptor>
                <transport_id>udp_transport</transport_id>
                <type>UDPv4</type>
                <!-- Reduce socket buffer size -->
                <sendBufferSize>4096</sendBufferSize>
                <receiveBufferSize>4096</receiveBufferSize>
                <!-- Reduce max message size, otherwise the participant creation fails -->
                <maxMessageSize>4096</maxMessageSize>
                <interfaceWhiteList>
                    <address>XXX.XXX.XXX.XXX</address>
                    <address>127.0.0.1</address>
                </interfaceWhiteList>
            </transport_descriptor>
        </transport_descriptors>

        <!-- Domain Participant Profile -->
        <participant profile_name="domainparticipant_profile_name" is_default_profile="true">
            <rtps>
                <!-- Use user defined UDPv4 transport -->
                <userTransports>
                    <transport_id>udp_transport</transport_id>
                </userTransports>
                <!-- Disable builtin transports -->
                <useBuiltinTransports>false</useBuiltinTransports>
            </rtps>
        </participant>

        <!-- Default publisher profile -->
        <data_writer profile_name="default_publisher_profile" is_default_profile="true">
            <topic>
                <!-- Tune initial allocations -->
                <resourceLimitsQos>
                    <max_samples>0</max_samples>
                    <allocated_samples>0</allocated_samples>
                </resourceLimitsQos>
            </topic>
        </data_writer>

        <!-- Default subscriber profile -->
        <data_reader profile_name="default_subscriber_profile" is_default_profile="true">
            <topic>
                <!-- Tune initial allocations -->
                <resourceLimitsQos>
                    <max_samples>0</max_samples>
                    <allocated_samples>0</allocated_samples>
                </resourceLimitsQos>
            </topic>
        </data_reader>
    </profiles>
</dds>
```

For the Turtlebot4, do the following:
1. SSH into the Turtlebot: `ssh ubuntu@<Ip address of raspberry pi>` (default password is turtlebot4)
2. Either modify `/etc/turtlebot4/fastdds_rpi.xml` (may get overridden with future Turtlebot4 updated) or create a new file (more configuration needed) and insert following configuration (replace XXX.XXX.XXX.XXX with IP-address of Raspberry Pi):
```bash
<?xml version="1.0" encoding="UTF-8" ?>
<dds>
    <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
        <!-- UDPv4 Transport profile -->
        <transport_descriptors>
            <transport_descriptor>
                <transport_id>udp_transport</transport_id>
                <type>UDPv4</type>
                <interfaceWhiteList>
                    <address>XXX.XXX.XXX.XXX</address>
                    <address>127.0.0.1</address>
                </interfaceWhiteList>
            </transport_descriptor>
        </transport_descriptors>
        
        <!-- Domain Participant Profile -->
        <participant profile_name="turtlebot4_default_profile" is_default_profile="true">
            <rtps>
                <!-- Use user defined UDPv4 transport -->
                <userTransports>
                    <transport_id>udp_transport</transport_id>
                </userTransports>
                <!-- Disable builtin transports -->
                <useBuiltinTransports>false</useBuiltinTransports>
            </rtps>
        </participant>
    </profiles>
</dds>
```
3. *Only if you created this as a new configuration*: 
   1. Go under ROS Setup -> Bash Setup -> Select RMW-Profile-fastrtps
   2. Insert full path to your created configuration file and save
   3. Back out to the main menu and apply settings

### Other untested, but possible solutions
1. Change RMW to CycloneDDS
2. Reduce the number of nodes which are visible in the network
3. Launch nodes sequentially

## Time synchronization
It is important for the Turtlebot's and all the clients that the time is synchronized. Otherwise you could run into problems. To achieve this NTP is utilized. All the clients (especially the Turtlebot) synchronizes with the host which runs the Sopias4-Fleetbroker. If the development container is used and the Turtlebot's are setup like instructed then everything should be setup already. However, if you run into time syncing issues, then it is useful to check if the NTP configuration is still working and if chrony (used as NTP server and clients) is running fine.