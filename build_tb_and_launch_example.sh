cd ~/catkin_ws/src
git clone https://github.com/bjortech/tb
cd ..
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
roscd tb_simulation
chmod +x scripts/tb_heightimg2pointcloud_node.py
chmod +x scripts/tb_simulate_surroundcloud_node.py
roscd tb_invoke
chmod +x tb_invoke_node.py
source ~/.bashrc
roslaunch tb_bringup tb_without_premade_octomap.launch
