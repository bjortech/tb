sudo apt-get install ros-melodic-oct* ros-melodic-move-base ros-melodic-dynamic-edt-3d ros-melodic-tf2* ros-melodic-hector-sensors-description ros-melodic-pointcloud-to-laserscan ros-melodic-map-server ros-melodic-dwa-local-planner -y
mkdir -p ~/tb_ws/src
cd ~/tb_ws/src
catkin_init_workspace  # initialize your catkin workspace
echo "source ~/tb_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
git clone https://github.com/bjortech/tb
cd ~/tb_ws
catkin_make
roscd tb_simulation
chmod +x scripts/tb_heightimg2pointcloud_node.py
chmod +x scripts/tb_simulate_surroundcloud_node.py
roscd tb_invoke
chmod +x tb_invoke_node.py
source ~/.bashrc
roslaunch tb_bringup tb_without_premade_octomap.launch
