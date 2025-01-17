# CSM Sim
This repo houses robot descriptions (for running `robot_state_publisher`) as well as configurations/assets for running Gazebo/Isaac Sim simulators with ROS2. For robot deployment, use the description-only branches so that simulator assets (~500MB) don't need to be pulled! Additionally, if adding a new robot description, use the `base` branch as a starting point.

## Setup/Usage
### Considerations
ROS2 is required to run either simulation, however, you must first consider which platform (and thus which ROS version) you plan to use:

| Simulator | Platform Support | Hardware Requirements | Additional Comments |
| - | - | - | - |
| [Isaac Sim](https://docs.omniverse.nvidia.com/isaacsim/latest/) | Ubuntu 22.04 / ROS2 Humble, WSL (and older versions of Ubuntu) | ⚠️ Minimum spec: NVIDIA RTX 3070, 32GB RAM, and a modern CPU with 4+ cores. All requirements can be found [here](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/requirements.html). | Provides a much more realistic simulation compared with Gazebo at the cost of higher hardware requirements and being closed source. If learning Isaac Sim for the first time (without an example), the learning curve is extremely intense! |
| [Gazebo](https://gazebosim.org/docs/latest/getstarted/) | **Natively** - Ubuntu 24.04 / ROS2 Jazzy, **Manual install** - Ubuntu 22.04 / ROS2 Humble | Anything modern will suffice. A discrete GPU will greatly speed up rendering. | Harmonic and up is required for this package to work properly, which greatly limits platform support. Core simulation (rigid body/collision) is comperable to Isaac Sim although advanced features are lacking and LiDAR simulation is much less realistic. Less of a learning curve compared to Isaac Sim and ROS2 integration is moderately more streamlined. |

**TLDR:** Currently, you cannot run both Isaac Sim and Gazebo (easily) on the same OS install, and should choose a platform that supports whichever simulator you prefer. *Note that WSL installs of either OS have not been tested with the project - this may or may not be a viable solution.*

**CSM Members:** unless you are doing advanced development with the LiDAR, use Ubuntu 24.04 and ROS2 Jazzy with Gazebo!

### Gazebo
To run this project, you will need a ROS2 and Gazebo installation. __Currently, the easiest and best pairing includes ROS2 Jazzy and Gazebo Harmonic (both LTS versions), but requires an Ubuntu 24.04 install (or equivalent).__ It is also possible to pair ROS2 Humble with Gazebo Harmonic on Ubuntu 22.04 if necessary, although this pairing is not officially supported and may require manual compilation of other simulation packages. Some additional considerations include:
- Running the simulator in WSL is nearly unusable due to performance reasons. A native install or dual-boot is recommended.
- If using WSL, Gazebo releases prior to Garden do not have functional hardware-accelerated graphics (and software rendering is unusable for this project).
- The TrackedVehicle plugin used by this project does not work out-of-the-box with Gazebo releases prior to Harmonic.
- ROS2 Jazzy is capable of being built from source on Ubuntu 22.04, however, rosdep will not be functional, meaning dependencies will also have to built from source.

#### Dependencies, Install, and Running a Standalone Simulation:

1. Follow the ROS2 install guide: [Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html), [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
    - To verify your installation, execute `printenv | grep ROS`.
2. Choose a Gazebo version to pair with ROS, and install [`ros_gz` bridge libraries](https://gazebosim.org/docs/harmonic/ros_installation) (this page lists all possible pairings and their level of support).
3. Install Gazebo: [Harmonic](https://gazebosim.org/docs/harmonic/install_ubuntu) (the release version can be changed using the dropdown menu)
    - To verify your installation, try running Gazebo using `gz sim`.
4. Run the following to ensure ros-gz dependencies are satisfied:
    ```bash
    sudo apt-get install ros-${ROS_DISTRO}-ros-gz-bridge ros-${ROS_DISTRO}-ros-gz-image ros-${ROS_DISTRO}-ros-gz-sim
    ```
5. Create a new ROS workspace and clone this repo:
    ```bash
    mkdir sim-ws && cd sim-ws
    mkdir src && cd src
    git clone https://github.com/Cardinal-Space-Mining/csm-sim -b lance-1-sim
    ```
6. Make sure all required packages are installed using rosdep:
    ```bash
    rosdep install --ignore-src --from-paths . -r -y
    ```
7. Build and source (execute when in workspace directory):
    ```bash
    colcon build --symlink-install
    source install/setup.bash
    ```
8. Run the simulation (uses an Xbox controller to drive the robot, and starts a Foxglove bridge node):
    ```bash
    ros2 launch csm_sim gz_sim.launch.py
    ```

### Isaac Sim
Make sure you have read the [considerations](#considerations) section above before continuing.

#### Dependencies, Install, and Running a Standalone Simulation:
1. Follow the ROS2 Humble [installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).
    - To verify your installation, execute `printenv | grep ROS`.
2. Follow the Issac Sim [installation guide](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_workstation.html).
    - The default install location is `~/.local/share/ov/pkg/isaac-sim-{SIM VERSION}/`. If this is not the case, you will need to specify your install location when using the launch script using the `isaac-root:={INSTALL LOCATION}` option (or modify the launch file). This will also be the case if you have installed a different version of the sim than the default (currently **4.2.0**).
3. Create a new ROS workspace and clone this repo:
    ```bash
    mkdir sim-ws && cd sim-ws
    mkdir src && cd src
    git clone https://github.com/Cardinal-Space-Mining/csm-sim -b lance-1-sim
    ```
4. Make sure all required packages are installed using rosdep:
    ```bash
    rosdep install --ignore-src --from-paths . -r -y
    ```
5. Build and source (execute when in workspace directory):
    ```bash
    colcon build --symlink-install
    source install/setup.bash
    ```
6. Launch the simulation (uses an Xbox controller to drive the robot, and starts a Foxglove bridge node):
    ```bash
    ros2 launch csm_sim isaac_sim.launch.py
    ```
    - It can take up to a few minutes for the gui to appear on a fresh install!
7. Press the play button on the left-hand side of the gui to play the simulation. Beware of stopping and restarting the sim as this may break other ROS nodes being run alongside the sim (sim clock gets reset).

## ROS2 Project Integration
An example project integrating this repo can be found [here](https://github.com/Cardinal-Space-Mining/lance-2025/tree/sim-remote).

The general premise is as follows:
1. Include this repo as a submodule alongside other packages in your project.
2. Run `colcon build` and `source install/setup.bash` as normal (from your workspace directory) to build and install.
3. Reference the `gz_sim.launch.py` or `isaac_sim.launch.py` launch files from your project's launch files to begin a simulation alongside other ROS2 nodes. Note the launch options for enabling/disabling the robot state publisher, joystick control, and the foxglove brige.

__*Last updated: 1/17/25*__
