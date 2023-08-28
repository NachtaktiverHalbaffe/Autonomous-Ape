# Troubleshooting
There a some known problems that can occur time to time. Following a overview of them and possible solutions are given.

## Startup of Navigation stack takes long or isn't starting
This is completly normal due to technical restrictions on the robot side. Long term this could get better with updates to the Turtlebot4 and Create3 platform. 
Until then, it could take sometimes 2-3 minutes. If it takes longer or something seems wrong, you could try the following:
- Check if Turtlebot is running correctly. You can see it in the LED ring on the powerbutton. It should normally be steady white. You can see the color coding at [https://iroboteducation.github.io/create3_docs/hw/face/](https://iroboteducation.github.io/create3_docs/hw/face/)
- Run `ros2 node list` and see if the nodes from the Robot show up. It should look like the following (namespaces can vary):
    ```Bash
    dsf
    ```
    If not, then you can use `restart_turtlebot_nodes.py` in the Python package `sopias4_framework.tools.scripts` to restart the nodes remotely until they show up. Note: After restarting it can take up to approx. 30 seconds until are nodes are shown

## The LED ring of the Turtlebot is showing red
If its pulsing red, then the Turtlebot needs to be charged. If it's showing solid/static red, then the Create 3 hang up and needs to be powercycled (complete shutdown and reboot). This can happen regulary because the Create3 has very limited CPU power 
an running Navigation2 the event communication could be enough to max out the processor and after some times the Create 3 can be unresponsive or be completly locked. In longterm this can be fixed by new firmware which optimizes load on the CPU, but 
until then you can try doing following:
- See if you updated the NTP configuration like mentioned in the Turtlebot4 installation guide in this documentation
- If you're implementation allows it: Start the components one after another instead all at once e.g first AMCL, then Rviz2 and then Navigation2