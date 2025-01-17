# Helper code for ROS1 Franka Interface

- All hardware for experiments run with ROS Melodic and Python 2 on a Franka Panda robot.

Note: all experiments can be run on a single computer if desired.


## Setup


1. Clone git repository or unzip source files into a catkin workspace (recommended location `$HOME/franka_ws/src`)

2. Initialize workspace by running `catkin_make` from `franka_ws` directory

Note 1: you may get some errors the first time since you haven't run rosdep yet. If so, run the next line then run this again.

Note 2: any time you add/remove files or edit C++ files, it's good practice to rerun `catkin_make`

3. Install ROS dependencies with `rosdep install franka_interface_ros1`

4. Install python dependencies by running `pip install -r requirements.txt`  from the `franka_interface_ros1` directory

5. If you want to run distributed training and/or use Intel's pytorch bindings, install optional python dependencies by running `pip install -r optional-requirements.txt`  from the `franka_interface_ros1` directory.

6. Source catkin workspace with `source devel/setup.bash`

7. If running on hardware, also be sure to set the `ROS_MASTER_URI` either on the command line or by sourcing `source robot.bash`


### Configuration-specific hard-coded references (mostly relevant for hardware testing):

| variable | notes | files |
| -- | -- | -- |
| robot_ip | ip address of your robot | start_robot.launch; franka_control.launch|
| machine tag | name/ip address of your robot-control computer | cam_monitor.launch |
| video device | /dev/video* where your webcam is mounted | cam_monitor.launch; run.launch; start_robot.launch |
| ROS_MASTER_URI | ip address of your robot | robot.bash |
| robot initial position | the initial / reset pose of the robot is hard coded. must change manually if you want a new start pose for either hardware or simulation | /scripts/go_vel; /scripts/pybullet_service |


## More Details

High level description of files included in this directory:

```bash
.
├── config
│   ├── cam.rviz                            # RVIZ hardware visualization
│   ├── franka_control.yaml                 # ROS hardware controller setup file
│   ├── gui.png                             # GUI display
│   └── panda_arm.urdf.xacro                # modified to add table
├── include                                 # C++ ROS Controllers
│   └── file names excluded, see directory
├── launch
│   ├── cam_monitor.launch                  # remote monitoring of hardware
│   ├── franka_control.launch               # sets up franka control configuration
│   ├── start_camera.launch                 # sets up camera
│   ├── start_gelsight.launch               # sets up gelsight
│   ├── start_robot.launch                  # sets up hardware interface
│   └── start_gelsight.launch               # sets up ultrasound imager
├── msg                                     # ROS message files
│   └── file names excluded, see directory
├── scripts
│   ├── conditional_brightness              # changes intrinsic webcam settings using v4l2-ctl
│   ├── gelsight_publisher                  # collects and publishes data from gelsight sensor(s)
│   ├── go_vel                              # connection between experiment commands and C++ controllers
│   ├── gui                                 # GUI for common ROS commands
│   ├── lamp_brightness                     # changes brightness of GVM LED lights (note: must be connected to computer via wifi)
│   ├── random_listener                     # publishes reset commands to prevent 'random' test methods from getting stuck
│   ├── ultrasound_publisher.py             # publishes ultrasound data from windows computer
│   ├── ultrasound_subscriber.py            # receives ultrasound data from windows computer
│   └── utils.py                            # helper functions
├── src                                     # C++ ROS Controllers
│   └── file names excluded, see directory
├── srv                                     # ROS Services
│   └── file names excluded, see directory
├── ati_telnet.expect                       # helper script to communicate with ATI F/T Sensor
├── CMakeLists.txt                          # ROS setup file
├── package.xml                             # ROS setup file
├── plugin.xml                              # ROS hardware controller setup file
├── requirements.txt                        # packages required to run this package
├── reset.sh                                # helper script to recover from franka motion errors (if not using GUI)
├── robot.bash                              # helper script to source robot for hadrware tests
├── setup.bash                              # helper script to source ros workspace and setup F/T sensor (if using /desired)
└── setup.py                                # helper script to set up python package via ROS

```

## References

Franka controller adapted from "ROS integration for Franka Emika research robots", GitHub repository, https://github.com/frankaemika/franka_ros.git
