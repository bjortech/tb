# tb
Generic Highlevel Autonomy Framework 

This is an attempt to share my work over the last years. The goal is to create a framework for totally isolated operation in threedimensional, uknown, dynamic and 
GPS-denied environments. Tests have been done with multiple configurations.

In adittion to working on live hardware I have shared my supersimple simulator. This "simulator" will work without anything other than an image file displaying height. 

An  example heightimage is included. 
# TB_BRINGUP:
After installing octomap, ros-packages, edto etc and building these packages, roslaunch tb_bringup tb2.launch should work. It probably doesn't, but I'll fix it. 

# TB_ROBOT: 
Robot description and publishing of transforms. 
Robot has two sensors connected on their respective servo(s). One tilt-joint connected with VLP-16 and one pan-tilt-joint with stereo camera. Configs are known to work,possible exception is the pan-tilt-slamdunk pipeline, as this has not been tested live. 

# TB_SIM: 
Sice latest tests have been with DJI, DJI-generic topics for attitude, altitude and position are used in the simulator. I have also created a joint_states publisher 
that will function as joint_state_publisher- but it will also respond to actuator commands (never figured how to do this without the GUI on JSP). 

# TB_CMD: 
This has been redone without the path-using option. I only include one way of cmd in initial pkg. 
#This is a setpoint publisher. To get the setpoints I have utilized two approaches. 
#1) Simulation of cmd_vel from move_base. 
#2) Use of "make_plan"-service rather than using move_base action. The planned path needs to be elevated and interpolated. 

# TB_PATHMAKER: 
This has been removed. This version only follows cmd_vel from move_base and elevation from tb_abmap. 
#Nodes for creation of 2d plan by move_base. Nodes for interpolation,elevation and smoothing of plan.  

# TB_INVOKE
-Not neccessary! Don't know how to delete
Node for receiving std_msgs::String msg that will be used as if in a terminal. Some commands are hard to automate through conventional API, this is a cheap workaround. 

# TB_ASSEMBLY:
Node that accumulate laser_assembler/AssembleScans2 for usage at desired rate. It also rewrites the frame_id of 2d_scans taken at current altitude for usage in the project 2d navigation. 

# TB_MBCLIENT: 
Move_base action client

# TB_FSM: 
Very basic state machine 
state==0: on ground 
state==1: in air, 
state==2 going home 
state==3 at takeoff xy coordinates

# TB_EDTOAUTO
This is the main source of input. Visualize with rviz (in tb_bringup) to see how it works. 
It consists of three nodes: 
# tb_edto_poly_node: 
Raycasts in a horisontal circle to find objects within proximity of setpoint
# TB_edto_down_node: 
Collects downwards points for clustering/segmentation. Finding roads or roofs. Packages working with this is not included atm. Targeter uses a simpified example function. 
# TB_edto_side_node: 
Collects points in plane within target distance of closest object (as given by EDTOoctomap.) This is to be able to trace the extents of the object in question, a basic inspection functionality. 

# TB_ABMAP
Frontier and elevator. Uses incoming assembly cloud to evaluate frontier and elevation

# TB_TARGETER
Simple targeter that tries to use identified vertical walls for local position reference, in abscence it uses nearest unknown position. All targets removed if close to any visited position. 





