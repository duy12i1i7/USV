# Virtual RobotX (VRX)
This repository is the home to the source code and software documentation for the VRX simulation environment, which supports simulation of unmanned surface vehicles in marine environments.
* Designed in coordination with RobotX organizers, this project provides arenas and tasks similar to those featured in past and future RobotX competitions, as well as a description of the WAM-V platform.
* For RobotX competitors this simulation environment is intended as a first step toward developing tools prototyping solutions in advance of physical on-water testing.
* We also welcome users with simulation needs beyond RobotX. As we continue to improve the environment, we hope to offer support to a wide range of potential applications.
## Now supporting Gazebo Sim and ROS 2 by default
We're happy to announce with release 2.0 VRX has transitioned from Gazebo Classic to the newer Gazebo simulator (formerly [Ignition Gazebo](https://www.openrobotics.org/blog/2022/4/6/a-new-era-for-gazebo)). 
* Gazebo Garden and ROS 2 are now default prerequisites for VRX.
* This is the recommended configuration for new users.
* Users who wish to continue running Gazebo Classic and ROS 1 can still do so using the `gazebo_classic` branch of this repository. 
  * Tutorials for VRX Classic will remain available on our Wiki.
  * VRX Classic will transition from an officially supported branch to a community supported branch by Spring 2023.
 
## Getting Started
This guide explains how to install dependencies needed to build and run VRX on your host machine.
### Preparing Your Host Machine
#### Step 1: Install ROS 2 Humble and Gazebo Garden.
Follow the installation instructions provided and return here when finished:
- [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- [Gazebo Garden](https://gazebosim.org/docs/garden/install_ubuntu)
#### Step 2: Install additional dependencies:
```
sudo apt install python3-sdformat13 ros-humble-ros-gzgarden ros-humble-xacro
```
### Installing VRX
Once you have set up your development environment, the following steps will download and build VRX:
1. Create a colcon workspace and clone the vrx repository
```
git clone https://github.com/duy12i1i7/USV.git
mv USV/vrx_ws ~/
```
2. Source your ROS 2 installation.
```
source /opt/ros/humble/setup.bash
```
3. Build the workspace
```
cd ~/vrx_ws
colcon build --merge-install
```
4. Setting up your environment
Now that you've built the simulation, you will need to source the setup script before you can do anything with it. From the root of your workspace, run:
```
. install/setup.bash
```
### Running VRX
This can take some time due to size of the environment. The first time this world is launched, it will also download 3D models from the [vrx collection on Fuel](https://app.ignitionrobotics.org/OpenRobotics/fuel/collections/vrx)
```
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
```
## Customizing the Roboboat
We recommend following the tutorials in order as each one builds off the last. At the end of these tutorials you will be familiar with how to work with your RoboBoat model in the VRX environment.
### Running the RoboBoat example world
The venue for the RoboBoat competition is Nathan Benderson Park, in Sarasota County, Florida, USA:
![image](https://github.com/user-attachments/assets/353b67bf-4e99-4a93-b399-221293aee340)
1. Running the empty world
This can take some time due to size of the environment. The first time this world is launched, it will also download 3D models from the [vrx collection on Fuel](https://app.ignitionrobotics.org/OpenRobotics/fuel/collections/vrx)
```
ros2 launch vrx_gz vrx_environment.launch.py world:=nbpark
```
2. Adding a vehicle
To open the world file with one of the example RoboBoat vehicles (or your own custom vehicle), you'll need to add it to the world. Open the `<YOUR_VRX_WORKSPACE/src/vrx/vrx_gz/worlds/npark.sdf` file from your VRX workspace with your favorite editor. Then add your custom roboboat model within the `<world>` tag:
```
<!-- RoboBoat 01 -->
<include>
  <name>roboboat01</name>
  <pose>-185 1088 0 0 0 0</pose>
  <uri>roboboat01</uri>
</include>
```
Recompile the project and run as follow:
```
cd ~/vrx_ws
colcon build --merge-install
. install/setup.bash
ros2 launch vrx_gz vrx_environment.launch.py world:=nbpark
```
### Teleoperate your vehicle
#### Thruster Setup
The actuators for the boat we spawned in the previous tutorial are defined by the Gazebo Sim Thruster Class. While there are a number of ways to control heading, with two thrusters we can simply employ differential thrust.
#### Launching Teleoperation Nodes
##### Launch the ROS/Gazebo bridge
To open the world file with one of the example RoboBoat vehicles (or your own custom vehicle), you'll need to add it to the world.  Open the `<YOUR_VRX_WORKSPACE/src/vrx/vrx_gz/worlds/npark.sdf` file from your VRX workspace with your favorite editor. Then add your custom roboboat model within the `<world>` tag:
```
<!-- RoboBoat 01 -->
<include>
  <name>roboboat01</name>
  <pose>-185 1088 0 0 0 0</pose>
  <uri>roboboat01</uri>
</include>
```
Recompile the project and run as follow:
```
cd ~/vrx_ws
colcon build --merge-install
. install/setup.bash
ros2 launch vrx_gz vrx_environment.launch.py world:=nbpark
```
To send messages from ROS2 to Gazebo Sim, we must utilize a bridge, specifying the topics and messages we need to send. 
- For the left thruster:
```
ros2 run ros_gz_bridge parameter_bridge /model/roboboat01/joint/left_engine_propeller_joint/cmd_thrust@std_msgs/msg/Float64]gz.msgs.Double
```
- and for the right thruster:
```
ros2 run ros_gz_bridge parameter_bridge /model/roboboat01/joint/right_engine_propeller_joint/cmd_thrust@std_msgs/msg/Float64]gz.msgs.Double
```
##### Test the Bridges
You can test that the bridges are working by publishing a topic to one or both of the thrusters:
```
ros2 topic pub /model/roboboat01/joint/left_engine_propeller_joint/cmd_thrust std_msgs/msg/Float64 "data: 150"
```
using this command you should see your boat going in circles.

### Customize your vehicle
- The first step is to prepare the graphic files for the boat model you want to use as a replacement for the default one. Make sure to include all necessary files, such as `model.dae`, `model.config`, `model.sdf`, and any other required resources to fully render the boat.

To ensure your model file is functioning correctly, navigate to the directory containing the graphic files, open a terminal, and run the following command to test:
```
gz sim -r model.sdf
```
If Gazebo launches and the boat is successfully displayed, congratulations! Your boat model files are ready. Once your model files are ready, create a new folder named `ship` and copy all your boat model files into this folder.

After preparing the `ship` folder, copy it into the `models` directory located at `~/vrx_ws/src/vrx/vrx_gz/models`.

- For now, this is a very similar model to the RoboBoat01 available in VRX. Let's test it. Open the `<YOUR_VRX_WORKSPACE/src/vrx/vrx_gz/worlds/npark.sdf` file from your VRX workspace with your favorite editor. Then add your custom roboboat model within the `<world>` tag:
```
<!-- My custom roboboat -->
<include>
  <name>my_roboboat</name>
  <pose>-175 1120 0 0 0 3.14</pose>
  <uri>ship</uri>
</include>
```
Here, `my_roboboat` is the name of your model as specified in the SDF file. For example: `<model name="my_roboboat">`.

- Go to your VRX workspace, recompile, and run Gazebo:
```
cd ~/vrx_ws
colcon build --merge-install
. install/setup.bash
ros2 launch vrx_gz vrx_environment.launch.py world:=nbpark
```
You should see your model successfully loaded within the Nathan Benderson park.
![image](https://github.com/user-attachments/assets/613d5775-6014-4b82-a3fd-4233b522e6d9)



