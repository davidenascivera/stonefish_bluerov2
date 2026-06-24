# Stonefish BlueROV2 ROS2 (fork)

Simulation of the Blue Robotics BlueROV2 in Stonefish, bridged to ArduSub SITL
through ROS2.

This is a fork of [bvibhav/stonefish_bluerov2](https://github.com/bvibhav/stonefish_bluerov2).
The bridge script, `scripts/ardusim_patch.py`, is the main file in this repo.
It's what actually talks to ArduPilot's JSON SITL backend, and the reason this
fork exists is that the upstream version of that script sends two physical
quantities (`accel_body`, `velocity`) in the wrong frame. See
[Changes from upstream](#changes-from-upstream) for details.

Tested on Ubuntu 22.04, ROS2 Humble, Stonefish 1.3.


## Run with Docker Compose

The `docker-compose.yml` brings up two containers: `ardusub` (ArduSub SITL) and
`stonefish` (the Stonefish simulator GUI).

**1. Set up the workspace folder**

```bash
mkdir -p ros2_ws_ardusub/src
cd ros2_ws_ardusub/src
git clone https://github.com/davidenascivera/stonefish_bluerov2.git
cd ..
```

**2. Copy the compose file to the workspace root**

`docker-compose.yml` ships inside the package (`src/stonefish_bluerov2/Docker/`),
but must sit at the root of `ros2_ws_ardusub/` (next to `src/`), since its build
context points at `./src/stonefish_bluerov2`:

```bash
cp src/stonefish_bluerov2/Docker/docker-compose.yml .
```

**3. Build the images (first time only)**

```bash
docker compose up --build
```

**4. Subsequent runs**

The Stonefish container opens an OpenGL/X11 window, so the host X server must let
the container connect. Grant access once per login, then start:

```bash
xhost +local:root      # let local root (the container user) reach the X server
docker compose up
```

## Compatibility

This bridge targets **Stonefish 1.3** and **stonefish_ros2 v1.3**. The
accelerometer fix in `ardusim_patch.py` assumes the IMU topic published by
`stonefish_ros2` does *not* include gravity (true for 1.3).

**If you move to Stonefish 1.5 or later, gravity is already included in the
IMU topic published by the bluerov2 node.** Applying this patch unmodified on
top of that will add gravity twice and break the EKF. Don't upgrade Stonefish
without revisiting this.

## Requirements

- ROS2 with colcon and the DDS backend (Humble tested)
- Stonefish simulator, `v1.3` branch
- stonefish_ros2 wrapper, `v1.3` branch
- ArduPilot SITL (ArduSub)

## Setup

### Option A: Docker

The `Dockerfile` in the repo root builds Stonefish, stonefish_ros2, and this
bridge into one image, but that's only the simulator side. You'll still need
ROS2 (for MAVROS), QGroundControl, and ArduPilot SITL installed on the host;
see [Running the simulation](#running-the-simulation) below for the actual
commands.

Build:
```bash
cd src/stonefish_bluerov2
docker build -t stonefish-bluerov2:latest .
```

Run (`--network host` is what lets `ros2 topic list` work from the host
machine):
```bash
xhost +local:docker   # allow X11 for the Stonefish GUI
docker run -it --rm \
  --network host \
  --env DISPLAY=$DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --gpus all \
  --device /dev/dri \
  --env NVIDIA_DRIVER_CAPABILITIES=all \
  stonefish-bluerov2:latest
```
Running the container launches the simulation automatically. If you need to
relaunch it manually from inside the shell, the `run-stonefish` alias does
the same thing (`ros2 launch stonefish_bluerov2 bluerov2_sim.py`).

If `ros2 topic echo` (or `list`) doesn't see anything from the host, even
with `--network host`, the DDS daemon on the host is often still trying
shared-memory transport, which doesn't cross the container boundary. Force
UDP and restart the daemon:
```bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 daemon stop
```
Then retry the `ros2 topic` command — the daemon restarts automatically and
picks up the new transport setting.

### Option B: Manual install

1. **ROS2**: follow the [official install guide for Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

2. **Stonefish**: clone and build [patrykcieslak/stonefish](https://github.com/patrykcieslak/stonefish),
   checking out `v1.3` *before* building. Later versions change what the IMU
   topic contains (see Compatibility above).

3. **stonefish_ros2**: set up a colcon workspace as described in the
   [colcon tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html),
   clone [patrykcieslak/stonefish_ros2](https://github.com/patrykcieslak/stonefish_ros2)
   into it, and check out `v1.3`.

4. **ArduPilot SITL**: follow the
   [ArduPilot Linux build guide](https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux):
   ```bash
   git clone --recurse-submodules https://github.com/ArduPilot/ardupilot
   cd ardupilot
   Tools/environment_install/install-prereqs-ubuntu.sh -y
   . ~/.profile
   ```
   Make sure the submodules actually came down. A shallow/partial clone is
   the most common reason this silently doesn't work. At this point
   [QGroundControl](https://qgroundcontrol.com/downloads/) should be able to
   connect to the (still unsimulated) vehicle.

5. Extra system packages this fork needed, that aren't always pulled in by
   the steps above:
   ```bash
   sudo apt install gcc-arm-none-eabi
   sudo apt install libglm-dev libsdl2-dev libfreetype-dev libgl-dev libglu1-mesa-dev
   sudo apt install ros-humble-mavros ros-humble-mavros-extras
   sudo apt install ros-humble-tf-transformations
   sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
   ```

6. If you have conda on the same machine, deactivate it before building or
   running anything ROS2-related, since the two environments conflict:
   ```bash
   conda deactivate
   ```

7. Clone this repo into your workspace and build:
   ```bash
   cd ~/ros2_ws/src
   git clone <this-repo-url>
   cd ~/ros2_ws
   colcon build --event-handlers console_direct+ --cmake-args --symlink-install --packages-select stonefish_bluerov2
   ```

## Running the simulation

### BlueROV2

Four terminals:

```bash
# 1: ArduSub SITL
sim_vehicle.py -v ArduSub --model JSON --map -L PHILL -m --streamrate=-1 --out udp:127.0.0.1:14551
```
This will complain about "link 1 down" until the simulator side connects.
That's expected at this point, not a problem.

```bash
# 2: Stonefish + BlueROV2
source ~/ros2_ws/install/setup.bash
ros2 launch stonefish_bluerov2 bluerov2_sim.py
```
If you didn't clone the repo locally and went with [Docker](#option-a--docker)
instead, skip this step — running the container already launches the
simulation automatically.

```bash
# 3: MAVROS
mavros_node --ros-args -p fcu_url:=udp://127.0.0.1:14551@
```

```bash
# 4: optional sanity check
ros2 topic list
ros2 topic echo /mavros/imu/data
```

In QGroundControl: vehicle setup → change the vehicle configuration to
**Vectored-6DOF**, and enable joystick input.

## Common problems

**Stonefish shows "No frame from SITL, is it running?"**: ArduPilot SITL is
probably not running, or hasn't connected yet. Check the ArduSub terminal.

**ArduSub SITL stays stuck waiting for a heartbeat**, something like:
```
Log Directory:
Telemetry log: mav.tlog
Waiting for heartbeat from tcp:127.0.0.1:5760
MAV> link 1 down
```
This means Stonefish isn't sending frames, for one of two reasons: either
something in a recent change broke the bridge, or there's a leftover instance
from a previous run still holding the port. Kill everything and start clean:
```bash
pkill -f "sim_vehicle|arducopter|ardusub|mavproxy|mavros_node|stonefish" ; sleep 2
```

## Changes from upstream and Reference frames

See `scripts/ardusim_patch.py` for the in-depth analysis of what changed and
why.

- ArduSim (ArduPilot's SITL) and Stonefish's world frame are both NED-like,
  with Z pointing down. This is the same convention behind the `accel_body`
  fix mentioned above.
- MAVROS, by default, converts most position/orientation topics to ENU
  before publishing them on ROS, so `/mavros/local_position/pose` and
  `/mavros/imu/data` are Z-up. Depth still comes out **negative** once the
  vehicle is underwater, and that's expected, not a bug: in an up-positive
  frame, being below the origin (roughly the surface) just means negative Z,
  same as a negative altitude.

## References

- ArduSub developer docs: https://www.ardusub.com/developers/developers.html
- Stonefish: https://github.com/patrykcieslak/stonefish
- stonefish_ros2: https://github.com/patrykcieslak/stonefish_ros2
- Upstream repo: https://github.com/bvibhav/stonefish_bluerov2
- BlueROV2 ROS2 driver: https://github.com/bvibhav/bluerov2_interface