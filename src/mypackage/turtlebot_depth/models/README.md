# Depth Anything V2 models

## Purpose

This folder stores Depth-Anything-V2 model weights used by the `turtlebot_depth` package. For RTAB-Map and
other SLAM systems you must use a *metric* depth model (provides real distances). Relative depth models only
provide scale-normalized depth and are not suitable for generating metric 3D point clouds.

## Recommended model (metric)

For indoor monocular SLAM we recommend the Metric Hypersim Small model. Download and place it in this
directory:

```bash
cd /home/ros/monocular_ws/src/src/mypackage/turtlebot_depth/models
wget https://huggingface.co/depth-anything/Depth-Anything-V2-Metric-Hypersim-Small/resolve/main/depth_anything_v2_metric_hypersim_vits.pth
```

Then launch using the model file explicitly if needed:

```bash
ros2 launch turtlebot_depth gazebo_with_depth.launch.py model_file:=/home/ros/monocular_ws/src/src/mypackage/turtlebot_depth/models/depth_anything_v2_metric_hypersim_vits.pth
```

## Encoder parameter

If you change to a different family of model, update the `encoder` in `config/depth_anything_params.yaml`:

- For ViT-S (small): `encoder: vits`
- For ViT-B (base): `encoder: vitb`
- For ViT-L (large): `encoder: vitl`

Use the metric Hypersim model for SLAM and point cloud generation.
