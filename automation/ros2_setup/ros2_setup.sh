# Ubuntu 20.04 ver
sudo apt update
sudo apt upgrade

sudo apt install python3-colcon-common-extensions
apt-cache policy | grep universe
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade
sudo apt install ros-foxy-desktop

echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=~/ros2_install" >> ~/.bashrc
printenv | grep -i ROS

cd ~/Desktop
mkdir "ros2_workspace"
cd ~/Desktop/ros2_workspace
mkdir "billee_ws"
cd ~/Desktop/ros2_workspace/billee_ws
mkdir "src"
cd ~/Desktop/ros2_workspace/billee_ws/src
ros2 pkg create billee_pkg --build-type ament_cmake

# cloning the correct dir
git clone https://github.com/jmoya34/BILL-EE
cd ~/Desktop/ros2_workspace/billee_ws/src/BILL-EE
git checkout ros_bille_pkg
git pull

#moving around files
mv '/home/moya/Desktop/ros2_workspace/billee_ws/src/BILL-EE/launch' '/home/moya/Desktop/ros2_workspace/billee_ws/src/billee_pkg'
mv '/home/moya/Desktop/ros2_workspace/billee_ws/src/BILL-EE/scripts' '/home/moya/Desktop/ros2_workspace/billee_ws/src/billee_pkg'
mv '/home/moya/Desktop/ros2_workspace/billee_ws/src/BILL-EE/CMakeLists.txt' '/home/moya/Desktop/ros2_workspace/billee_ws/src/billee_pkg'
mv '/home/moya/Desktop/ros2_workspace/billee_ws/src/BILL-EE/package.xml' '/home/moya/Desktop/ros2_workspace/billee_ws/src/billee_pkg'
rm '/home/moya/Desktop/ros2_workspace/billee_ws/src/BILL-EE'

# Build Package
cd ~/Desktop/ros2_workspace
colcon build

echo "run source install/setup.bash to finish setting up terminal"