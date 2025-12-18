# TurtleBot Depth Setup Instructions

## 1. Download the Depth Anything V2 Model

Navigate to the models directory and download the model:

```bash
cd /home/ros/monocular_ws/src/src/mypackage/turtlebot_depth/models

# Depth-Anything V2 Metric Hypersim Small (vits)
wget -O depth_anything_v2_metric_hypersim_vits.pth \
  "https://huggingface.co/depth-anything/Depth-Anything-V2-Metric-Hypersim-Small/resolve/main/depth_anything_v2_metric_hypersim_vits.pth?download=true"

# Depth-Anything V2 Metric Hypersim Base (vitb)
wget -O depth_anything_v2_metric_hypersim_vitb.pth \
  "https://huggingface.co/depth-anything/Depth-Anything-V2-Metric-Hypersim-Base/resolve/main/depth_anything_v2_metric_hypersim_vitb.pth?download=true"

# Depth-Anything V2 Metric VKITTI Small (vits)
wget -O depth_anything_v2_metric_vkitti_vits.pth \
  "https://huggingface.co/depth-anything/Depth-Anything-V2-Metric-VKITTI-Small/resolve/main/depth_anything_v2_metric_vkitti_vits.pth?download=true"

# Depth-Anything V2 Metric VKITTI Base (vitb)
wget -O depth_anything_v2_metric_vkitti_vitb.pth \
  "https://huggingface.co/depth-anything/Depth-Anything-V2-Metric-VKITTI-Base/resolve/main/depth_anything_v2_metric_vkitti_vitb.pth?download=true"
```

## 2. Build the Package

```bash
cd /home/ros/monocular_ws
colcon build --packages-select turtlebot_depth
source install/setup.bash
```

## 3. Launch the Simulation with Depth Estimation

```bash
ros2 launch turtlebot_depth gazebo_with_depth.launch.py log_level:=debug
```

## Optional Launch Arguments

- `model_file`: Path to a different model file
- `x_pose`: Initial X position of the robot (default: 0.0)
- `y_pose`: Initial Y position of the robot (default: 0.0)
- `log_level`: Logging level for depth_anything node (default: info)

Example with custom arguments:
```bash
ros2 launch turtlebot_depth gazebo_with_depth.launch.py x_pose:=1.0 y_pose:=1.0 log_level:=debug
```

## Verify Depth Output

Check that the depth topic is being published:
```bash
ros2 topic list | grep depth
ros2 topic echo /camera/depth
```

## Topics

- Input image: `/camera/image_raw`
- Output depth: `/camera/depth`
- Camera info: `/sim_camera/camera_info`
