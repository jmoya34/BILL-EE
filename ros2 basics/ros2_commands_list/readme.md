# List of Commands

## Setting up packages

### Rebuilding package
```bash
colcon build
source install/setup.bash
```

<br>

## Nodes
### Checking nodes
```bash
ros2 pkg executables <your_pkg>
```

### Running node
```bash
ros2 run <your_pkg> <node>.py
```

<br>

## Interfaces
### Interface list
```bash
ros2 interface list
```

<br>

## Topics
### Topic list
```bash
ros2 topic list
```

### Echo topic
```bash
ros2 topic info <topic>
```


<br>

## Parameters
### Param list
```bash
ros2 param list
```
### Get param
```bash
ros2 param get <node_name> <param_name>
```
### Set Param
```bash
ros2 param set <node_name> <param_name> <value>
```

<br>

## Launch Files
### Run launch file
```bash
ros2 launch <pkg_name> <launch_file_name>
```