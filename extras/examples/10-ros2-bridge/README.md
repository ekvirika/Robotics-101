# 10 · ROS 2 bridge

**Not run on this machine.** Needs a sourced ROS 2 installation, turtlesim, and the dora ROS 2 bridge, which is experimental in 1.1.0.

```bash
source /opt/ros/jazzy/setup.bash
ros2 run turtlesim turtlesim_node      # in another terminal
cd examples/10-ros2-bridge && dora run dataflow.yml
```

- **`ros2:` in the YAML** makes dora start a bridge node, the Python nodes contain no ROS 2 code
- **`cmd_bridge`** publishes the driver's speed commands on `/turtle1/cmd_vel`, the turtle drives in circles
- **`pose_bridge`** subscribes to `/turtle1/pose` and hands it to the printer as an Arrow struct
- **Messages are one-row Arrow structs,** fields named like the ROS 2 message: `linear.x`, `angular.z`, `x`, `y`, `theta`
- **Example use:** keep Nav2 driving the base in ROS 2, run the camera and AI model in dora, connect them here
