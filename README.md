# QUICKLAUNCH
tb_installscripts contains scripts for installation of ros and for building all packages in a new workspace. Details in README at tb_installscripts. 
 
# ABOUT
Generic Highlevel Autonomy Framework 

This is an attempt to share my work over the last years. The goal is to create a framework for totally isolated operation in threedimensional, uknown, dynamic and 
GPS-denied environments. Tests have been done with multiple configurations.

In adittion to working on live hardware I have shared my supersimple simulator. This "simulator" will work without anything other than an image file displaying height. 

NB: Please understand that this is very basic and that there are a lot of bugs and errors.My hope is that someone would like to join me in replacing the different parts of the system with best-practice implementation, as well as defining and completing the missing components. 

NB2: As this is self-learned along the way, coding style is probably confusing to experienced coders. Feel free to help me convert to correct format. My nodes are all built by same source and with all dependencies, naturally a lot of redundant ones. All nodes are written in the same document, not using any includes or classes. 

Video of tests (look below the actual location, I have ran this package at a lower altitude (the test failed and it flew far too high):
https://www.youtube.com/watch?v=EKpb4s75W5g

# TB_BRINGUP:
Launch files and configurations for octomap,filters,move_base,map_server etc. 

# TB_ROBOT: 
Robot description and publishing of transforms. 
Robot has two sensors connected on their respective servo(s). One tilt-joint connected with VLP-16 and one pan-tilt-joint with stereo camera. Configs are known to work,possible exception is the pan-tilt-slamdunk pipeline, as this has not been tested live. 

# TB_SIMULATE: 
Sice latest tests have been with DJI, DJI-generic topics for attitude, altitude and position are used in the simulator. I have also created a joint_states publisher 
that will function as joint_state_publisher- but it will also respond to actuator commands (never figured how to do this without the GUI on JSP). 

# TB_CMD: 
This has been redone without the path-using option. I only include one way of cmd in initial pkg. 
#This is a setpoint publisher. To get the setpoints I have utilized two approaches. 
#1) Simulation of cmd_vel from move_base. 
#2) Use of "make_plan"-service rather than using move_base action. The planned path needs to be elevated and interpolated. 

# TB_PATHMAKER: 
This has been excluded. This version only follows cmd_vel from move_base and elevation from tb_abmap. 
#Nodes for creation of 2d plan by move_base. Nodes for interpolation,elevation and smoothing of plan.  

# TB_INVOKE
-Not neccessary! Don't know how to delete
Node for receiving std_msgs::String msg that will be used as if in a terminal. Some commands are hard to automate through conventional API, this is a cheap workaround when working on live hardware. 

# TB_ASSEMBLY:
Node that accumulate laser_assembler/AssembleScans2 for usage at desired rate. It also rewrites the frame_id of 2d_scans taken at current altitude for usage in the project 2d navigation. 

# TB_MBCLIENT: 
Move_base action client

# TB_FSM: 
Very basic finite state machine 
state==0: on ground 
state==1: in air, 
state==2 going home 
state==3 at takeoff xy coordinates

# TB_EDTOAUTO
This is an automated collection of environment coordinates created for visualization mainly. One of the nodes is bugged. 
The system is designed to use this as targeting input, but there are some challenges that are better saved for after understanding the rest of the system.

# TB_ABMAP
Frontier and elevator. Uses incoming assembly cloud to evaluate frontier and elevation. Other versions of the nodes uses EDTO, but the CPU-load will become too much. These nodes don't need to be perfectly accurate to function. 

# TB_TARGETER
Simple targeter that tries to use identified vertical walls for local position reference, in abscence it uses nearest unknown position from the frontier node. All targets removed if close to any visited position. This node is not working properly, but it still works as a simple demonstration of the system.



