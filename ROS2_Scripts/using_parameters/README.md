# Comands used in linux ubuntu terminal

```bash
ros2 param list
ros2 param get /<node_name> <param_name>
ros2 param set /<node_name> <param_name> <value>
```

## Using parameters in Python3
```python
self.declare_parameter("param_name", set_default_value)
self.get_parameter("param_name").get_parameter_value()
```
