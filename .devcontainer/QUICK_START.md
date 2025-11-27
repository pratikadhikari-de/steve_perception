# DevContainer Quick Start Guide

## ✅ Status: READY TO USE

This workspace uses a pre-built Docker image from GitHub Container Registry.

## 🚀 Quick Start for New Users

### Option 1: VSCode DevContainer (Recommended)
```bash
# 1. Clone the repository
git clone <repository-url>
cd steve_ros2_ws

# 2. Open workspace in VSCode
code .

# 3. Reopen in Container
# Press Ctrl+Shift+P or F1
# Select "Dev Containers: Reopen in Container"
# VSCode will automatically pull the image!
```

### Option 2: Pull and Run Image Directly
```bash
# Pull the pre-built image
sudo docker pull ghcr.io/riddheshmore/ros2-mmo700:latest

# Run interactively
sudo docker run -it --rm \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $(pwd):/home/ws \
  ghcr.io/riddheshmore/ros2-mmo700:latest bash
```

### Option 3: Build Locally (Advanced)
Only needed if you want to rebuild the image from scratch:

```bash
cd .devcontainer

docker build -f Dockerfile \
  --build-arg USERNAME=$(whoami) \
  -t custom_ros2:latest .
```

## 🔧 Common Issues

| Problem | Solution |
|---------|----------|
| Docker daemon not running | `sudo systemctl start docker` |
| Permission denied | `sudo usermod -aG docker $USER` then logout/login |
| Can't find base image | Check internet connection |
| X11 forwarding fails | Run `xhost +local:docker` on host |
| CMake cache errors | Run `rm -rf build install log` inside container |

## 📋 What's Included

✅ ROS2 Humble Desktop Full  
✅ MoveIt2  
✅ Gazebo  
✅ Navigation2  
✅ SLAM Toolbox  
✅ ros2_control  
✅ All project dependencies  

## 📝 Recent Changes

**2025-11-27 - Migrated to GHCR**
1. ✅ Docker image now hosted on GitHub Container Registry
2. ✅ Image: `ghcr.io/riddheshmore/ros2-mmo700:latest`
3. ✅ No local build needed - image pulls automatically
4. ✅ Colleagues can use the workspace immediately after clone

**Previous - Dockerfile Improvements**
1. ✅ Fixed `InvalidDefaultArgInFrom` warning - Moved ARG before FROM
2. ✅ Fixed `LegacyKeyValueFormat` warning - Updated ENV syntax
3. ✅ Build completes with ZERO warnings


