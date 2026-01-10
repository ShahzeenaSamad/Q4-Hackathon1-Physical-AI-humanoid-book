# ROS 2 Command Reference

A quick cheatsheet for the most common ROS 2 commands used throughout this textbook.

---

## 🌎 Workspace Management
```bash
# Build workspace
colcon build

# Build specific package
colcon build --packages-select <pkg_name>

# Source workspace
source install/setup.bash
```

## 📦 Package Operations
```bash
# Create new package
ros2 pkg create --build-type ament_python <pkg_name>

# List packages
ros2 pkg list
```

## 🟢 Node Operations
```bash
# Run a node
ros2 run <pkg_name> <node_name>

# List running nodes
ros2 node list

# Get info about a node
ros2 node info <node_name>
```

## 📡 Topic Operations
```bash
# List all topics
ros2 topic list

# Echo messages on a topic
ros2 topic echo <topic_name>

# Get topic type and count
ros2 topic info <topic_name>

# Publish to a topic manually
ros2 topic pub <topic_name> <msg_type> "<data>"
```

## 🛠️ Service Operations
```bash
# List services
ros2 service list

# Call a service
ros2 service call <service_name> <srv_type> "<data>"
```

## 🎬 Action Operations
```bash
# List actions
ros2 action list

# Send action goal
ros2 action send_goal <action_name> <action_type> "<data>"
```

## 📊 Useful GUI Tools
```bash
# Visualize node graph
rqt_graph

# Universal debugging tool
rviz2

# Plot topic data
rqt_plot
```
