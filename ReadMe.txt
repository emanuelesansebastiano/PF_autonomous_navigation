# PF_autonomous_navigation
Algorithms to navigate an underwater autonomous vehicle along a underwater pipe.



According to the scenario (static or dynamic) the underwater vehicle can be control by velocity or thrust input.
A selection menu will appear at the beginning of the execution.

The vehicle might be controlled by:
1) Manual input [PF_teleop_sansebastiano.py]
2) Waypoints input [PF_waypoints_sansebastiano.py]

3) Vision input [PF_vision_sansebastiano.py]


In order to visualize the post-process image of the camera [PF_vision.py]

DO NOT FORGET to make executable all the .py scripts by the following command string:
chmod +x ~src/"script".py

Required packages:
UWSIM
pipefollowing


Usage:
run in different shells:
- roscore
- roslaunch pipefollowing "scenario".launch
- rosrun PF_autonomous_navigation PF_"script".py

ENJOY!
