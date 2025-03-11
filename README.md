# uav-rotor-control

Simulation for low-level UAV control with Gazebo and PX4 designed for the Aerial Robotics laboratory classes.

Software version: 
- **ROS 2:** Humble [[docs](https://docs.ros.org/en/humble/index.html)]
- **Gazebo:** Harmonic [[docs](https://gazebosim.org/docs/harmonic/getstarted/)]
- **PX4:** 1.15 [[docs](https://docs.px4.io/v1.15/en/)]

> [!NOTE]
> This repository is prepared for working with Ubuntu OS. 
> If you are using Windows, please open Ubuntu in WSL 2.

## Prerequisites

First of all, before you start anything, please fork this repository and **work on your own copy**.

The best way to work with this repository is to use the prepared Docker image.
Follow the official [Docker Documentation](https://docs.docker.com/) to install the required software (note that on Linux, only the Docker Engine is required).
Also, if you have an Nvidia GPU and want to use its computing power within Docker, you should install the [Nvidia Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) (without experimental packages and rootless mode).

> [!NOTE]
> If you do not have an Nvidia GPU card, you need to modify the [devcontainer.json](.devcontainer/devcontainer.json) and set `"dockerComposeFile"` to `compose.nogpu.yaml`.

## Docker setup

> [!TIP]
> In Windows, it is recommended to use WSL2 with a native Linux file system.
> Normally directories from Windows are mounted as `/mnt` inside Ubuntu, but this may cause some problems due to the NTFS file system and permissions.
> Therefore, copy repository or clone it directly into `$HOME` directory.

Open the project's directory in VS Code.
In the bottom left corner, click on the icon with the two arrows pointing at each other.
The menu with different options will appear at the top - choose **"Open folder in container... "** and wait, the docker will be created.
Note that this may take some time (depending on your internet connection).

In terminal inside docker container, run:

```bash
cd /home/developer/ros2_ws
./setup.sh
sudo ./build_uXRCE.sh
./build.sh
source install/setup.bash
```

> [!NOTE]
> In Windows, there may be a problem with running bash scripts - two possible reasons are:
>
> 1. Scripts are not set to be executable files - you can change it with `sudo chmod +x script.sh`.
> 2. Wrong line end character - change `CRLF` to `LF` in the bottom right part of the VS Code GUI.

Above commands built your workspace with all needed dependencies.

### First run

Before running tests, the PX4 SITL (_Software in the Loop_) must be properly configured to work seamlessly with ROS2.

Firstly, you need to update the topics exchanged between PX4 and ROS2:

```bash
cp ~/ros2_ws/src/dds_topics.yaml ~/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml
```

Now you can build the PX4 framework for the SITL:

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

> [!NOTE]
> Since above command builds the whole PX4 framework, it may take some time.

Once built, the Gazebo window should appear with the drone inside.
In the terminal you will see some information about PX4 working in the background.
Wait until it prints out the message `[commander] Ready for takeoff!`.
This means that everything went well and PX4 is ready to go.

However, it is not yet ready to cooperate with ROS2 in the task of low-level UAV control.
To achieve this, you need to adjust some PX4 parameters.
You can do this using the same terminal that now acts as the PX4 console.
First, switch off the time synchronisation between the Gazebo and ROS2 time domains (ROS2 will use the Gazebo clock directly):

```bash
param set UXRCE_DDS_SYNCT 0
```

Secondly, switch off the disarming after landing detection (the PX4 landing detection system is sometimes too sensitive, which can lead to accidental disarming in flight):

```bash
param set COM_DISARM_LAND -1.0
```

Thirdly, change the vertical speed threshold to 0.0 m/s to completely disable landing detection in the PX4:

```bash
param set LNDMC_Z_VEL_MAX 0.0
```

Now turn off the PX4 SITL (`CTRL+C`) in order to save the configuration.
You are ready to run your code.

### Remarks on working with repository

Create a new ROS2 package and fill it with some valuable content :smile:.

When working with the ROS workspace, it is sometimes necessary to rebuild the whole workspace, e.g. if you want to run some script from the newly created package.
You can do this by running the [build.sh](.devcontainer/build.sh) script located in the `ros2_ws` directory inside docker.
Remember to also source the `~/ros2_ws/install/setup.bash` script.
Note that this is added to `~/.bashrc` so that it is automatically sourced when a new terminal is created.

For your convenience, the default ROS log path has been changed to `~/ros2_ws/src/logs`.
This will allow you to access them from outside the container.

## Additional features

### QGroundControl

QGroundControl is software designed to communicate with the PX4 running on board the UAV.
It allows access to the current state of the vehicle, internal autopilot parameters and logs collected during the flight.
To use it, download the AppImage according to the [instruction](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html#ubuntu) and run it with the following command:

```bash
./QGroundControl.AppImage --appimage-extract-and-run
```

To download logs from the UAV you need to run both px4_sitl and QGroundControl (in separate terminals).
In QGroundControl, left click (LMB) on the logo (top left corner) and select `Analyze Tools`.
There you will see the `Log Download` window with the list of all available data (if the list is empty, you can try clicking the `Refresh` button).
Here you can select the desired file and download it to the OS directory.

### Data exchange between Gazebo and ROS2

Natively, Gazebo and ROS have a similar internal structure with topics and services to transport data, but they are not seamlessly interoperable (e.g. different types).
However, there is an additional [ros_gz](https://github.com/gazebosim/ros_gz) package provided by Gazebo developers, which facilitates integration between Gazebo and ROS.
It is already installed in the docker, so you can use all the features you need.

For your convenience, there is also an [evs_gz_utils](./evs_gz_utils/) package in the repository.
You can read its own (not very rich - still under development) documentation to learn more.

### Rosbag

If you are running nodes inside ROS2, you can easily record all the data that is exchanged via topics.
Just use the [rosbag2](https://github.com/ros2/rosbag2) which is already installed in the docker.
Remember that you can manually select the topics you want to record and set the path where all the bags will be stored.

### PlotJuggler

If you've downloaded some logs from PX4 and recorded some rosbags, you'd like to be able to read and analyse them now, wouldn't you?
Fortunately, there is a good piece of software called [PlotJuggler](https://plotjuggler.io/) that is already installed in the docker.
You can run it with the following command:

```bash
ros2 run plotjuggler plotjuggler 
```
After some random meme, you will see the main window where you can import and plot data.
Some additional functions can be found in the linked documentation.

## FAQs

### But why?

[📢 Bad joke alert]
Because you want to pass the subject and get your degree, right?

### I started the Gazebo, but simulator window did not pop up

Re-open the folder locally and give additional permissions for Docker:
``` bash
xhost +local:docker
```

Re-open the folder in the container and check that it is now working correctly.
