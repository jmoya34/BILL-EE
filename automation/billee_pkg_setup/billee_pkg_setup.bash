WHOAMI=$(whoami)

cd ~/Desktop
mkdir "billee_ws"
cd ~/Desktop/billee_ws
mkdir "src"
cd ~/Desktop/billee_ws/src
ros2 pkg create billee_pkg --build-type ament_cmake

# cloning the correct dir
git clone https://github.com/jmoya34/BILL-EE
cd ~/Desktop/billee_ws/src/BILL-EE
git checkout ros_bille_pkg
git pull

#moving around files
mv "/home/${WHOAMI}/Desktop/billee_ws/src/BILL-EE/scripts" "/home/${WHOAMI}/Desktop/billee_ws/src/billee_pkg"
echo "IMPORTANT]manually remove /home/${WHOAMI}/Desktop/billee_ws/src/BILL-EE"


# File path
file_path="/home/"$WHOAMI"/Desktop/billee_ws/src/billee_pkg"

# Changing XMl script
search_xml_string="<buildtool_depend>ament_cmake<\/buildtool_depend>"
input_xml_var="<buildtool_depend>ament_cmake<\/buildtool_depend>\n  <buildtool_depend>ament_cmake_python<\/buildtool_depend>\n  <depend>rclpy<\/depend>"

if ! grep -q "<depend>rclpy<\/depend>" "${file_path}/package.xml"; then
    sed -i "s/${search_xml_string}/${input_xml_var}/" "${file_path}/package.xml"
fi

# Changing Cmake script
search_cmake_string="# uncomment the following section in order to fill in"
input_cmake_var="find_package(ament_cmake_python REQUIRED)\nfind_package(rclpy REQUIRED)\nament_python_install_package(scripts\/)"

# Automatically searches for all the scripts and addes them to CMake file
cd "/home/${WHOAMI}/Desktop/billee_ws/src/billee_pkg/scripts"
scripts=""
for item in $(ls)
do
    scripts+="scripts\/$item\n"
done
billee_files="$scripts"

install_part="\ninstall(PROGRAMS\n${billee_files}\nDESTINATION lib\/\${PROJECT_NAME}\n)"
sed -i "s/${search_cmake_string}/${input_cmake_var}${install_part}/" "${file_path}/CMakeLists.txt"

# Build Package
cd ~/Desktop/billee_ws
colcon build
echo "run source install/setup.bash to finish setting up terminal"
