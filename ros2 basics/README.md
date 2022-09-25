# Setting up ROS2 (Ubuntu 20.04 Distro)
A timeline on how to learn ROS2. I will be referring to the official [ROS2 documentation](https://docs.ros.org/en/foxy/index.html) as well as [Raymond Andrade's ROS2 robotics developer Course.](https://www.udemy.com/course/ros2-robotics-developer-course-using-ros2-in-python/)


## Installation
An advantage of ROS2 over ROS1 is the compatibility with Windows, Mac, and Linux. The path I recommend when working with robotics is Linux due to jetson and raspberry pi series using Linux. If you have a Mac or Windows device trying to follow Linux guide, I recommend using a virtual machine. Go into [Virtual Machine setup](VirtualMachineSetup)
* Here is the official documentation for installing [ROS2 Foxy Distro](https://docs.ros.org/en/foxy/Installation.html)
* After you finish installing ROS2 Foxy Distro run the following command so you do not have to source ros2 everytime you open a terminal
```bash
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
```

## Framework Overview
ROS offers a Data Distribution service (DDS) which is the communication pipeline interfaces with all executing code. The executing files using ROS functionality are known as **Nodes**

### Nodes have 3 ways of communication:
* **Publisher** nodes sending information that allow for **Subscriber** nodes to receive information.
* **Services** sends a request to a **Service server** that when completes a request will send back a response
* **Topics** are the individual communication pipelines. They are a parameter used in publisher and subscriber methods. Publisher create the topics, and subscribers call to the topic.
* **Actions** will send a goal which the **Actions server** will process the goal and send progress updates to the client that sent the goal. This process is known as **Feedback**.

### Node Parameters:
* Parameters are usefull for taking change into account instead of editing the source code and recompiling the project. 
* Parameters can be edited by **the user** OR **other nodes** to change parameters.

### Bag files:
* Bag files are usefull when you want to collect data.
* You are able to play back the information.
* A usefull example of this would be recording videos.

### Packages:
* Packages contain all code for a robots functionality.
* Packages can be shared between users to replicate.
* You are able to incorporate other packages along with your own as well.

## Creating Workspaces
Your starting directory doesn't matter. I personally like seeing my files on the desktop so I create my workspaces as such on Linux.

```bash
moya@:~/Desktop/ros_workspace$
```

Create workspaces inside the **ros_workspace directory** labeled as anything  you wish, and I will label mine as **learning_ws**. Inside the learning_ws include a new directory called src. Inside the src folder, run the following ros2 command to create a package
```bash
moya@:~/Desktop/ros_workspace/learning_ws/src$ ros2 pkg create learning_ws_pkg --build-type ament_cmake
```
**Note: you should see a folder with the package name and the following folders appear inside the package folder**

![folders](/ros2%20basics/imgs/amentcmake.png)

### Create Scripts folder:
* In the same directory as the cmakelists.txt file, create a folder for future scripts that we are going to make.

### Building package using colcon:
* Going back to our workspace directory in terminal, we are going to use colcon to build our package. For reference, the path should look something like this:
```bash
moya@:~/Desktop/ros_workspace/learning_ws$
```

* **[IMPORTANT]** Every time we adding a script or make a change to a script we have to rebuild the package using colcon.
* The following command is used to install colcon on Ubuntu
```bash
sudo apt install python3-colcon-common-extensions
```
* Using colcon is as simple as running the command
```bash
moya@:~/Desktop/ros_workspace/learning_ws$ colcon build
```
* There should be 3 new folders labeled
1. build
2. install
3. log
* To include the package to the terminal environment use commands
```bash
moya@:~/Desktop/ros_workspace$ source install/setup.bash
```

## For every new terminal run commands
```bash
source /opt/ros/foxy/setup.bash
colcon build
source install/setup.bash
```


## Configuring Packages
Everytime we add a new node to our package we have to configure out cmakelist.txt and package.xml file so the complier involves the newly created scripts.

1. In package.xml add the new following lines under line 10
```xml
<buildtool_depend>ament_cmake_python</buildtool_depend>
<depend>rclpy</depend>
```
![Visual Representation of how package.xml might look](/ros2%20basics/imgs/packxml.png)

2. Everytime we add a new node, we must write it down in CMakeLists.txt. Under line 19 in the CMakeLists.txt file write the following lines of code
```cmake
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
ament_python_install_package(scripts/)
```


* **[IMPORTANT]** Colcon build will fail if you do not create a __init__.py file inside scripts file!
* Using the command will allow your nodes to be included as such. There is no limit to the amount of nodes you wish to add. In the example below, I have a publisher and subscriber node inside my scripts folder that I am attempting to associate with my package.
```cmake
install(PROGRAMS
scripts/publisher.py
scripts/subscriber.py
DESTINATION lib/${PROJECT_NAME}
)
```
![Image on how CMakeLists.txt should look](/ros2%20basics/imgs/cmake_txt.png)
* Please note that if the top of the python script does not involve a ```#! /usr/bin/env python3``` then colcon report an error. Look at examples for more info.

3. By going back into workspace directory we can colcon and source the package 
```bash
moya@:~/Desktop/ros_workspace$ colcon build
moya@:~/Desktop/ros_workspace$ source install/setup.bash
```
* To check if it properly included the scripts as nodes run the following command
```bash
ros2 pkg executables <your_pkg>
```
<br>

# Developing ROS2 skills in python
All nodes consist of the same format for publisher, subscribers, and publisher subscribers in combination. Look into [script_examples folder](/script_examples) for references.

<br>
As Discussed earlier publishers share data with other nodes and along with subscribers, and are essential to understand.

<br>

## Publishers

### Importing modules
```python
#! /usr/bin/env python3
# note that i wrote this in ubuntu 20.04.4 where i use colcon to build packages 
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
```
* **[NOTE]:** In this instance we are publishing a String data type, and if we were to attempt to upload a different data type there would be an error. To include more data types import the next to String. Checking what types are allowed are simple as googling it or typing in the terminal the following command
```bash
ros2 interface types
```

### Writing a publisher Node as a class
```python
class HelloWorldPublisher(Node):
    def __init__(self):
        super().__init__("Hello_world_pub_node") # This is the name of the node. No spaces are allowed in the name. Adding spaces can lead to issues.
        self.pub = self.create_publisher(String, 'hello_world', 10)# We put the data, the name of the topic, and the QOS which stands for quality of service
        self.timer = self.create_timer(2, self.publish_hello_world) #The parameters take the time to repeat the function and the function being run.

        self.counter = 0 

    def publish_hello_world(self):
        msg = String() # We create a string message
        msg.data = 'Hello World ' + str(self.counter) # Then we load the message
        self.pub.publish(msg) # this then publishes the message
        self.counter += 1
```

### Running the class in main
```python
def main():
    rclpy.init()
    my_pub = HelloWorldPublisher()
    print("publisher Node Running...")

    try:
        rclpy.spin(my_pub)
    except KeyboardInterrupt:
        my_pub.destroy_node()
        rclpy.shutdown


if __name__ == '__main__':
    main()
```

From here add the node to the cmakelist.txt file and build with colcon then source is. 


## Subscribers
### Importing Modules
```python
#! /usr/bin/env python3
# note that i wrote this in ubuntu 20.04.4 where i use colcon to build packages
 
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
```
We are receiving a String from the publisher node therefore we are going to need the String interface type.

### Writing a subscriber node as a class
```python
 '''
create_subscription has 4 parameters. The variable type, topic name WHICH MUST MATCH WITH PARAMETER TOPIC NAME, callback function which will be run everytime something is published over a topic, and QOS (quality of service)
'''
class HelloWorldSubscriber(Node):
    def __init__(self):
        super().__init__("hello_world_sub_node")
        self.sub = self.create_subscription(String, "hello_world",
                                            self.subscriber_callback, 10)
       
    
    def subscriber_callback(self, msg):
        print("received: " + msg.data) # msg.data is the information from the pub
```

### Running the class in main
```python
def main():
    rclpy.init()

    my_sub = HelloWorldSubscriber()

    print("Waiting for data to be published over topic")

    try:
        rclpy.spin(my_sub)
    except KeyboardInterrupt:
        my_sub.destroy_node()
        rclpy.shutdown


if __name__ == '__main__':
    main()
```

## Publisher and Subscriber combination

```python
#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class pubAndSub(Node):
    def __init__(self):
        super().__init__("publisher_subscriber_node")
        self.sub = self.create_subscription(String, "hello_world", self.subscriber_callback, 10)
        self.pub = self.create_publisher(String, "hybrid_node", 10)

    def subscriber_callback(self, msg):
        new_msg = String()
        new_msg.data = msg.data + " plus new words are being added in"
        print(new_msg.data)
        self.pub.publish(new_msg)

def main():
    rclpy.init()

    my_sub = pubAndSub()
    print("Waiting for data to be published over topic")

    try:
        rclpy.spin(my_sub)
    except KeyboardInterrupt:
        my_sub.destroy_node()
        rclpy.shutdown


if __name__ == '__main__':
    main()
```
Please note that your publisher node also has to be running in order for this to work.
* Please note how in this instance where we publish information that it doesn't not require using the timer method as before. 

<br>

## Parameters
Messing around with parameters can be done through both python and terminal. Inside the [ros2_commands_list](/ros2_commands_list) you can see the list of parameter commands you can use in the terminal. Using it through python involves 3 steps:
1. Declare the parameter
2. Get the parameter
3. Get parameter values
```python
class testingParameters(Node):
    def __init__(self):
        super().__init__("testing_parameters")

        self.default_value = 10
        self.declare_parameter("wheel_radius", default_value)
        self.sub = self.create_subscription(String, "hello_world", selfsubscriber_callback, 10)

    def create_subscription(self, msg):
        print(self.get_parameter("wheel_radius").get_parameter_value())
```
* We can see that **declare_parameter** takes two inputs. The name of parameter we choose, and the value we assign it. 
* To access the parameter info we have to use the **get_parameter** and **get_parameter_value** method.

<br>

# Launch Files
Create a launch folder in the same directory as the scripts and cmake directory. Inside create a python file with **.launch.py** at the end of the file name. The entire script example is located inside [script_examples folder.](/script_examples/launch/publisher_node.launch.py) 

1. Import the following modules 
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
```
2. Create the function as follows
```python
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_learning_pkg',
            executable='publisher.py',
            name="hello_world_node"
        ),
        ExecuteProcess(
            cmd=['ros2', 'topic', 'list'],
            output='screen'
        )
    ])
```
* The follow are parameters that can be used inside Node
![Image of Node parameters](/ros2%20basics/imgs/launch_parameters.png)
* Execute Process allows us to run terminal commands through python. Words must be separated as shown in the generate launch description function.

3. We need to add to the CMakeLists.txt file in order to be able to colcon build in directory. Add the following code under:
```Cmake
install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}/
)
```
![Image of how adding launch to cmake looks like](/ros2%20basics/imgs/launch_cmake.png)
We can now colcon build and source the package to be able to execute the launch file. The following command are colcon building and sourcing follows as:
```bash
ros2 launch <pkg_name> <launch_file_name>
```

# Packages
## Using other developers packages

Method 1: Installing to ros path

* Using image_common repo as an example
https://github.com/ros-perception/image_common
* We use "-" to represent a space and underscore in the name
```bash
sudo apt install ros-$ROS_DISTRO-image-common
```
- The folder locates to path **/opt/ros/foxy/share**

Method2: Adding package to workspace
* We clone the repo into the workspace folder inside of src 
* You can find the package name often inside of the xml file
```bash
colcon build --package-select <package_name>
```

<br>

# Services
## Interfaces with request, response architecture 

### Understanding services:
* Open a terminal and run
```bash
ros2 interface list
```
* Using the particular interface std_srvs/srv/SetBool we can see how services are formatted when shown
```bash
ros2 interface show std_srvs/srv/SetBool
```
* The message should display as:
```
bool data # e.g. for hardware enabling / disabling
---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages
```
* There are two sections with the top being the request and the bottom is the response

### Creating services:
* In your package, create a folder labled srv.
![Image of how all folders together look like](/ros2%20basics/imgs/pkg_files_img.png)
* Inside the folder we create the file, and for this example we will create an odd or even checker.
* Create a .srv file called OddEvenChecker.srv
* The top line of the .srv file is the request type and the bottom is the response which is seperated by three dashes. The request and responses are giving names like variables.
```srv
int64 number
---
string decision
```




* Note you can get syntax coloring by installing the follow extension on vscode:
![Extension name: Msgs Language Support](/ros2%20basics/imgs/srv_extension.png)