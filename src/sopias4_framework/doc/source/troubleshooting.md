# Troubleshooting
There are some known problems that can occur time to time. Following a overview of them and possible solutions are given.

## WSL2 doesnt connect to Turtlebot
First check if you have followed each step of the installation command correctly. 

After that, make sure that you repeat step 4 from the Windows installation guide everytime when you restart your Windows/WSL2 and not only one time at the installation.

## Startup of Navigation stack takes long or isn't starting
This is completly normal due to technical restrictions on the robot side. Long term this could get better with updates to the Turtlebot4 and Create3 platform. 

Until then, it could take sometimes 2-3 minutes. You can determine if the startup failed or some problem occurred when it deviates significantly from this behavior:
1. The localization (AMCL) and the navigation (Navigation) should be started. This should be visible. Also Rviz is launched for visualization
2. After that the Turtlebot locates itself after some time. If it has itself located, then a cluster of green little arrows occur in Rviz
3. After that, the costmaps comes online:
   - It waits until all TF buffers received all the needed transformations. During this process, the following could be seen in the logs: 
   ```bash
   [controller_server-3] [INFO] [turtle1.local_costmap.local_costmap]: Timed out waiting for transform from base_link to odom to become available, tf error: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist
   ```
   - After all Transformations are received, then the local costmap show up
   - The same is repeated for the global costmap
4. After that the rest should be up recently fast and the following logs should appear in the terminal: 
    ```bash
    [lifecycle_manager-4] [INFO]  Managed nodes are active
    [lifecycle_manager-4] [INFO]  Creating bond timer...
    ```

If it takes longer or something seems wrong, you could try the following:
- Check if Turtlebot is running correctly. You can see it in the LED ring on the power button. It should normally be steady white. You can see the color coding at [https://iroboteducation.github.io/create3_docs/hw/face/](https://iroboteducation.github.io/create3_docs/hw/face/). A red light is usually a sign that the Turtlebot need to be powercycled
- Run `ros2 node list` and see if the nodes from the Robot show up. It should look like the following (namespaces can vary):
    ```Bash
        /turtle1/analyzers
        /turtle1/joint_state_publisher
        /turtle1/joy_linux_node
        /turtle1/oakd
        /turtle1/oakd_container
        /turtle1/robot_state_publisher
        /turtle1/rplidar_composition
        /turtle1/teleop_twist_joy_node
        /turtle1/turtlebot4_base_node
        /turtle1/turtlebot4_diagnostics
        /turtle1/turtlebot4_node
    ```
    If not, see [Turtlebot4 nodes doesn't show up](#heading-target)


## The LED ring of the Turtlebot is showing red
If its pulsing red, then the Turtlebot needs to be charged. 

If the robot is rebooting/starting, then this could temporary be normal. Just wait if it fixes itself. Usually after 3-5 minutes the Turtlebot should have been started and the red LED ring disappeared.

If it's showing solid/static red, then the Create 3 hang up and needs to be powercycled (complete shutdown and reboot). This can happen regularly because the Create3 has very limited CPU power 
an running Navigation2 the event communication could be enough to max out the processor and after some times the Create 3 can be unresponsive or be completely locked. In long term this can be fixed by new firmware which optimizes load on the CPU, in short term you can look into the fixes which are described in the advanced information.

(heading-target)=
## Turtlebot4 nodes doesn't show up
Often this is due to some networking quirks. It can be fixed by either running the `restart_turtlebot()` utility inside `sopias4_framework.tools.scripts.turtlebot_ssh_scripts.py` (can sometimes fail) or by:
1. SSH into terminal: `ssh ubuntu@<ip address of Raspberry Pi of Turtlebot>` (default password is turtlebot4)
2. Run `turtlebot4-service-restart` in the terminal
3. Wait some time and wait
4. Run `exit` to close the SSH connection