WHOAMI=$(whoami)

cd ~/Desktop
ws_name=""
read -p "Enter name of workspace folder: " ws_name

mkdir "$ws_name"
cd ~/Desktop/"$ws_name"
mkdir "src"
cd ~/Desktop/"$ws_name"/src

pkg_name=""
read -p "Enter name of package: " pkg_name

ros2 pkg create "$pkg_name" --build-type ament_cmake

# Downloading script files
cd "/home/${WHOAMI}/Desktop/$ws_name/src/$pkg_name"
mkdir "scripts"
cd "/home/${WHOAMI}/Desktop/$ws_name/src/$pkg_name/scripts"
echo "$(curl –o https://raw.githubusercontent.com/jmoya34/ROS2-Basic-Necessities/main/script_examples/publishers/publisher.py)" > publisher.py
echo "$(curl –o https://raw.githubusercontent.com/jmoya34/ROS2-Basic-Necessities/main/script_examples/subscribers/subscriber.py)" > subscriber.py
echo "" > __init__.py

# File path
file_path="/home/"$WHOAMI"/Desktop/$ws_name/src/$pkg_name"

# Changing XMl script
search_xml_string="<buildtool_depend>ament_cmake<\/buildtool_depend>"
input_xml_var="<buildtool_depend>ament_cmake<\/buildtool_depend>\n  <buildtool_depend>ament_cmake_python<\/buildtool_depend>\n  <depend>rclpy<\/depend>"

if ! grep -q "<depend>rclpy<\/depend>" "${file_path}/package.xml"; then
    sed -i "s/${search_xml_string}/${input_xml_var}/" "${file_path}/package.xml"
fi

# Changing Cmake script
ros_verion="$(echo $ROS_DISTRO)"
amentChange="\/"
if "$ros_verion" == "humble"
then
    amentChange=""
fi

search_cmake_string="# uncomment the following section in order to fill in"
input_cmake_var="find_package(ament_cmake_python REQUIRED)\nfind_package(rclpy REQUIRED)\nament_python_install_package(scripts$amentChange)"

# Automatically searches for all the scripts and addes them to CMake file
cd "/home/${WHOAMI}/Desktop/$ws_name/src/$pkg_name/scripts"
scripts=""
for item in $(ls)
do
    scripts+="scripts\/$item\n"
done
billee_files="$scripts"

install_part="\ninstall(PROGRAMS\n${billee_files}\nDESTINATION lib\/\${PROJECT_NAME}\n)"
sed -i "s/${search_cmake_string}/${input_cmake_var}${install_part}/" "${file_path}/CMakeLists.txt"

# Build Package
cd ~/Desktop/"$ws_name"
colcon build
echo "run source install/setup.bash to finish setting up terminal"
