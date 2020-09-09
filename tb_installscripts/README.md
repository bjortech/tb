# QUICKLAUNCH
(Only tested once. If the process stops, find the line where it failed and copy paste one by one.)
 
Case A) Fresh install Ubuntu 18.04 (must be this version!).
  - Download the file "install_ros.sh"
  - open a terminal -> go to the folder where the file install_ros.sh is and write
	 chmod +x install_ros.sh 
	./install_ros.sh
  
Case B) Robot Operating System (Melodic) is installed or .install_ros.sh is complete
  - Download the file "build_and_launch.sh"
  - open a terminal -> go to the folder where the file build_and_launch.sh is and write
	 chmod +x build_and_launch.sh 
	./build_and_launch.sh


NB: This will create a new workspace and source that workspace in your .bashrc. This is intended for beginners who don't want their ws flooded with these packages. 
Note that this change in .bashrc must be removed to not automatically source tb_ws. To do this: 
  - open a terminal -> write
	sudo nano ~/.bashrc
	-> then move to the end of the document
	remove or comment out (#in front) the following line:  source ~/tb_ws/devel/setup.bash
  	save changes (ctrl + x -> y -> enter) 
	(The changes will be in effect next terminal you open)

	

