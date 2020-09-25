sudo apt-get install ros-melodic-oct* ros-melodic-move-base ros-melodic-dynamic-edt-3d ros-melodic-tf2* ros-melodic-hector-sensors-description ros-melodic-pointcloud-to-laserscan ros-melodic-map-server ros-melodic-dwa-local-planner -y
mkdir -p ~/tb_ws/src
cd ~/tb_ws/src
catkin_init_workspace  # initialize your catkin workspace
echo "source ~/tb_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
git clone -b freak_del5 https://github.com/bjortech/tb
cd ~/tb_ws
catkin_make
mkdir -p ~/brain
source ~/tb_ws/devel/setup.bash
roslaunch tb_bringup freak.launch
