software_driving_interface
==========================

Handles communication between hardware driving interface (HDI) and gazebo simulation. Abbreviated SDI.

Communication:
HDI --> SDI:	HDI_cmd messages
SDI --> Sim:	drc_vehicle_xp900/key/cmd, drc_vehicle_xp900/hand_brake/cmd, drc_vehicle_xp900/gas_pedal/cmd, drc_vehicle_xp900/hand_wheel/cmd, drc_vehicle_xp900/brake_pedal/cmd, drc_vehicle_xp900/direction/cmd
Sim --> SDI:	drc_vehicle_xp900/key/state, drc_vehicle_xp900/hand_wheel/state, ground_truth_odom
SDI --> HDI:	HDI_state messages

Scripts:
Contains python scripts for testing HDI<-->SDI communication

Msgs:
cmd and state messages for HDI<-->SDI communication

Launches:
Launch files for the SDI and HDI.

roslaunch software_driving_interface SDI.launch
