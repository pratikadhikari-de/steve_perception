# Docker Image Management

## Current Setup

The devcontainer is configured to use the public Docker image from GitHub Container Registry:

**Image:** `ghcr.io/riddheshmore/ros2-mmo700:latest`

This image contains the current state of the ROS2 workspace with all packages pre-built.

## For Colleagues: Getting Started

### Option 1: Using VS Code DevContainer (Recommended)

1. **Clone the repository:**
   ```bash
   git clone <repository-url>
   cd steve_ros2_ws
   ```

2. **Open in VS Code:**
   ```bash
   code .
   ```

3. **Reopen in Container:**
   - Press `F1` or `Ctrl+Shift+P`
   - Select: "Dev Containers: Reopen in Container"
   - VS Code will automatically pull the image and start the container!

**That's it!** The Docker image will be pulled automatically from GitHub Container Registry.

### Option 2: Run Docker Directly

```bash
# Pull the image
docker pull ghcr.io/riddheshmore/ros2-mmo700:latest

# Run interactively
docker run -it --rm \
  -v $(pwd):/home/ws \
  --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ghcr.io/riddheshmore/ros2-mmo700:latest bash
```

### Option 3: Using Local Tar File (if image is not public)

If the GHCR image is private and you received the tar file:

```bash
docker load -i ros2_mmo700.tar
```

### Using with VS Code DevContainer

1. Make sure the image is loaded (check with `docker images`)
2. Open the workspace in VS Code
3. Press `F1` → "Dev Containers: Reopen in Container"
4. The devcontainer will use the saved image directly (no rebuild needed!)

### Updating the Image

If you make changes and want to commit them:

1. Find your running container ID:
   ```bash
   docker ps
   ```

2. Commit the changes:
   ```bash
   docker commit <container_id> test_ros2_sim_gazebo:saved_state
   ```

3. (Optional) Save to tar file:
   ```bash
   docker save -o ros2_mmo700.tar test_ros2_sim_gazebo:saved_state
   ```

### Switching Back to Dockerfile Build

Edit `.devcontainer/devcontainer.json`:
- Comment out the `"image"` line
- Uncomment the `"build"` section

## Image Contents

This image includes:
- ROS2 Humble Desktop Full
- All workspace packages built
- Navigation2, SLAM Toolbox
- MoveIt2
- Gazebo with ros2_control
- Custom robot descriptions (MMO-700, etc.)
- All dependencies and configurations

## Notes

- The image preserves the current state of `/home/ws/install`
- Workspace source files are mounted from the host, so code changes are reflected immediately
- Only the installed packages and system packages are in the image
