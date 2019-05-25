# handover_moveit

The code here is meant to reproduce the experiment conducted in the handover release paper by Zhao Han and Holly Yanco, The Effects of Proactive Release Behavior during Human-Robot Handovers, accepted by HRI 2019.

## Hardware

- [Rethink Baxter robot](https://www.rethinkrobotics.com/baxter/)
- [PhidgetInterfaceKit 8/8/8](https://www.phidgets.com/?tier=3&catid=2&pcid=1&prodid=18)
- [Voltage Divider](https://www.phidgets.com/?tier=3&catid=49&pcid=42&prodid=92)
- [Force Sensing Resistor - Interlink Electronics 1.5″ Square 20N ](https://www.phidgets.com/?tier=3&catid=6&pcid=4&prodid=209)
- 2 USB extension cables
  1. Connect PhidgetInterfaceKit to a PC
  2. Connect Voltage Divider to FSR, hide the hardware behind the right shoulder of Baxter

## Software/Dependencies

- Ubuntu 14.04
- [ROS indigo](http://wiki.ros.org/indigo/Installation/Ubuntu)
- [Baxter SDK](http://sdk.rethinkrobotics.com/wiki/Home)
- [moveit!](http://sdk.rethinkrobotics.com/wiki/MoveIt_Tutorial#Installation.2FPrerequisites) -- select the indigo branch in the git repo
- [libphidget22](https://www.phidgets.com/docs/OS_-_Linux)
- Python 2 (Pre-installed with Ubuntu 14.04; make sure Python 3 binary is not in your path)
- Ruby 2.4.4 (any newer version or some old versions compatible with 1.9 should also work)
- [Recommended] [JetBrains CLion IDE](https://www.jetbrains.com/clion/)

## Run the experiment

Make sure the absolute path `/home/zhao/Dropbox/research17fall/experiment/` in all files is changed to yours.

Make sure the IP address of Baxter in the `run_experiment/run.rb` is yours.

Enable the robot: `rosrun baxter_tools enable_robot.py -e`

`cd` to the root of this project

Launch the .launch files: `roslaunch handover_moveit baxter_get_ready.launch`. The baxter_get_ready.launch is in the `launch` directory, you can take a brief look at what it is doing.

get almost ready: `rosrun handover_moveit get_almost_ready_node`

look at participants: `rosrun handover_moveit look_at_participants_node`

run the experiment: `ruby run_experiment/run.rb`
