# ROS2 MMO-700 Workspace - Quick Start

## For New Users / Colleagues

This workspace is pre-configured with a Docker container that includes all dependencies and packages.

### Prerequisites

- Docker installed
- VS Code with "Dev Containers" extension (for VS Code method)
- Git

### Quick Start (3 steps!)

#### 1. Clone the repository
```bash
git clone <your-repository-url>
cd steve_ros2_ws
```

#### 2. Open in VS Code
```bash
code .
```

#### 3. Reopen in Container
- Press `F1` (or `Ctrl+Shift+P`)
- Type and select: **"Dev Containers: Reopen in Container"**
- Wait for the Docker image to pull (first time only, ~5-10 minutes)

**Done!** You now have a fully configured ROS2 environment!

---

## What's Included in the Docker Image?

The Docker image (`ghcr.io/riddheshmore/ros2-mmo700:latest`) includes:

✅ **ROS2 Humble** (Desktop Full)  
✅ **Navigation2** & **SLAM Toolbox**  
✅ **MoveIt2** for motion planning  
✅ **Gazebo** with ros2_control  
✅ **Custom robot descriptions** (MMO-700, UR arms, Robotiq grippers)  
✅ **All workspace packages pre-built**  

---

## Running Simulations

Once inside the container:

### Launch the MMO-700 Robot in Gazebo

Run the following commands inside the container:

```bash
# inside container
cd /home/ws

# update submodules if needed
git submodule update --init --recursive --force || true

source /opt/ros/humble/setup.bash

# Install dependencies
rosdep install --from-paths src/neo_simulation2 --ignore-src -r -y || true

# Build the simulation package
colcon build --packages-select neo_simulation2 --symlink-install

source install/setup.bash

# Launch the simulation
ros2 launch neo_simulation2 simulation.launch.py    my_robot:=mmo_700 world:=neo_workshop arm_type:=ur5e include_pan_tilt:=true
```

### Teleop Control
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### View in RViz
```bash
rviz2
```

---

## Building Packages

If you modify source code:

```bash
cd /home/ws
colcon build --packages-select <package_name>
source install/setup.bash
```

---

## Alternative: Run Without VS Code

If you prefer command line:

```bash
docker pull ghcr.io/riddheshmore/ros2-mmo700:latest

docker run -it --rm \
  -v $(pwd):/home/ws \
  --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --gpus all \
  ghcr.io/riddheshmore/ros2-mmo700:latest bash
```

---

## Troubleshooting

### Image is private / access denied
If the Docker image is private, you'll need to:
1. Get a GitHub Personal Access Token from an admin
2. Login to GHCR:
   ```bash
   docker login ghcr.io -u Riddheshmore
   # Use the PAT as password
   ```

### Display issues with GUI
Make sure X11 forwarding is enabled:
```bash
xhost +local:docker
```

---

## More Information

- See `.devcontainer/DOCKER_IMAGE.md` for Docker management details
- See `.devcontainer/QUICK_START.md` for devcontainer specifics
- Check `src/` directory for package source code

---

## Contact

For issues or questions, contact the repository maintainer.
