# Depth Anything V2 Models

## Download Instructions

Download the Depth Anything V2 model from Hugging Face and place it in this directory.

### Available Models

Choose one of the following models based on your needs:

1. **Small (ViT-S)** - Fastest, lowest accuracy
   ```bash
   wget https://huggingface.co/depth-anything/Depth-Anything-V2-Small/resolve/main/depth_anything_v2_vits.pth
   ```

2. **Base (ViT-B)** - Balanced performance
   ```bash
   wget https://huggingface.co/depth-anything/Depth-Anything-V2-Base/resolve/main/depth_anything_v2_vitb.pth
   ```

3. **Large (ViT-L)** - Best accuracy, slower
   ```bash
   wget https://huggingface.co/depth-anything/Depth-Anything-V2-Large/resolve/main/depth_anything_v2_vitl.pth
   ```

### Quick Download (Recommended: Small model)

From this directory, run:
```bash
cd /home/ros/monocular_ws/src/src/mypackage/turtlebot_depth/models
wget https://huggingface.co/depth-anything/Depth-Anything-V2-Small/resolve/main/depth_anything_v2_vits.pth
```

### Using a Different Model

If you download a different model variant, update the `encoder` parameter in `config/depth_anything_params.yaml`:
- For ViT-S: `encoder: vits`
- For ViT-B: `encoder: vitb`
- For ViT-L: `encoder: vitl`

And specify the model file when launching:
```bash
ros2 launch turtlebot_depth gazebo_with_depth.launch.py model_file:=/path/to/your/model.pth
```
