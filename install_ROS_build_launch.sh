
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-melodic-desktop-full
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt install python-rosdep
sudo rosdep init
rosdep update
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace  # initialize your catkin workspace
git clone https://github.com/bjortech/tb
cd ..
catkin_make
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
roscd tb_simulation
chmod +x scripts/tb_heightimg2pointcloud_node.py
chmod +x scripts/tb_simulate_surroundcloud_node.py
roscd tb_invoke
chmod +x tb_invoke_node.py
source ~/.bashrc
roslaunch tb_bringup tb_without_premade_octomap.launch
