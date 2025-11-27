# Docker Quick Start Guide

## ✅ Status: TESTED & WORKING

The Dockerfile has been tested and verified to work correctly.

## 🚀 Quick Start (3 Options)

### Option 1: VSCode DevContainer (Easiest)
```bash
# 1. Open workspace in VSCode
# 2. Press Ctrl+Shift+P
# 3. Select "Dev Containers: Reopen in Container"
```

### Option 2: Manual Build
```bash
cd /path/to/steve_ros2_ws

docker build -f .devcontainer/Dockerfile \
  --build-arg USERNAME=$(whoami) \
  -t steve_ros2:latest .
```

### Option 3: Run Existing Image
```bash
docker run -it --rm \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --gpus all \
  --mount type=bind,source=$(pwd),target=/home/ws \
  steve_ros2:latest bash
```

## 🔧 Common Issues

| Problem | Solution |
|---------|----------|
| Docker daemon not running | `sudo systemctl start docker` |
| Permission denied | `sudo usermod -aG docker $USER` then logout/login |
| Can't find base image | Check internet connection |
| X11 forwarding fails | Run `xhost +local:docker` on host |

## 📋 What's Included

✅ ROS2 Humble Desktop Full  
✅ MoveIt2  
✅ Gazebo  
✅ Navigation2  
✅ SLAM Toolbox  
✅ ros2_control  
✅ All project dependencies  

## 📝 Changes Made (2025-11-27)

1. ✅ Fixed `InvalidDefaultArgInFrom` warning - Moved ARG before FROM
2. ✅ Fixed `LegacyKeyValueFormat` warning - Updated ENV syntax
3. ✅ Build now completes with ZERO warnings

## 📞 Need Help?

See detailed troubleshooting in `DOCKER_TEST_RESULTS.md`
