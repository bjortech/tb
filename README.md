# tb
Generic Highlevel Autonomy Framework 

This is an attempt to share my work over the last years. The goal is to create a framework for totally isolated operation in threedimensional, uknown, dynamic and 
GPS-denied environments. Tests have been done with multiple configurations. While it may look very crude, this setup actually works. 

In adittion to working on live hardware I have shared my supersimple simulator. This "simulator" will work without anything other than an image file displaying height. 
An  example heightimage is included. 

# GOAL: 
The goal with this system is to enable and share a very simple way of utilizing ROS for autonomy. My hope is that we can end up creating a free, generic solution that can be used by anyone who needs to inspect or maniupalte anything, anywhere. 

# TB_ROBOT: 
Robot description and publishing of transforms. 
Robot has two sensors connected on their respective servo(s). One tilt-joint connected with VLP-16 and one pan-tilt-joint with stereo camera. Configs are known to work,possible exception is the pan-tilt-slamdunk pipeline, as this has not been tested live. 

# TB_SIM: 
Sice latest tests have been with DJI, DJI-generic topics for attitude, altitude and position are used in the simulator. I have also created a joint_states publisher 
that will function as joint_state_publisher- but it will also respond to actuator commands (never figured how to do this without the GUI on JSP). 

# TB_CMD: 
This is a setpoint publisher. To get the setpoints I have utilized two approaches. 
1) Simulation of cmd_vel from move_base. 
2) Use of "make_plan"-service rather than using move_base action. The planned path needs to be elevated and interpolated. 

# TB_PATHMAKER: 
Nodes for creation of 2d plan by move_base. Nodes for interpolation,elevation and smoothing of plan.  

# TB_INVOKE
Node for receiving std_msgs::String msg that will be used as if in a terminal. Some commands are hard to automate through conventional API, this is a cheap workaround. 

# TB_ASSEMBLY:
Node that accumulate laser_assembler/AssembleScans2 for usage at desired rate. It also rewrites the frame_id of 2d_scans taken at current altitude for usage in the project 2d navigation. 

# TB_MBCLIENT: 
Move_base action client

# TB_FSM: 




