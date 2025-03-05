# Update function
## About ROS2 control
Est-ce que l'on peut contrôler intégralement (déplacement + canon) le bateau avec ROS2 ?
Si oui faire un launcher ros2 pour pouvoir le lancer avec un launcher globale par équipe.
Si non faire les controllers necessaires (via bridge ros2gz ou custom code) et Faire un code où la vitesse max de déplement du bateau et du canon sont paramètrables (pour équilibrer la partie)
#### About creating a launcher to control the entire ship (movement + weapons) using ROS2

I’ve set up a launcher that integrates all the necessary nodes into a single script: `launch_simulation.sh`. You can check all the changes directly in the corresponding GitHub commit.

To summarize the update — controlling the ship is now simplified. All you need to do is publish to specific ROS2 topics to command each engine individually. Here’s a concrete example:

After running `launch_simulation.sh` with the required parameters, controlling the ship’s engines becomes straightforward. Simply open a new terminal and run the following command to control the **left engine**:

```bash
ros2 topic pub /model/{warship_id}/joint/left_engine_propeller_joint/cmd_thrust std_msgs/msg/Float64 "data: 150"
```

This immediately spins the left engine at **150 rad/s**, causing the ship to move in an arc, driven by this engine alone.

Similarly, if you want to control the **right engine**, just publish to this topic:

```bash
ros2 topic pub /model/{warship_id}/joint/right_engine_propeller_joint/cmd_thrust std_msgs/msg/Float64 "data: 150"
```

This unified approach makes it much easier to control both propulsion systems directly through ROS2 topics, streamlining the process for future integration with higher-level control systems.

In addition to the ship control launcher, I’ve also made significant improvements to the `cannon.py` script, which handles turret rotation and firing. Previously, the cannon would instantly fire a projectile directly to the target point. Now, we’ve introduced a new parameter called `max_speed_rocket`.

This parameter controls not only how far the projectile can travel, but also its speed. In other words, the projectile’s flight characteristics now depend directly on this configurable speed limit.

For a detailed breakdown of these changes, you can refer to the latest commits in the `cannon.py` file.

To using it like a topic, we define a node named `warship_control`, which public the topic to control the gun and the rocket on each warship, here is how can we use this topic:
```bash
ros2 topic pub /fire warship_control/FireCommand "{source: 'enemy_ship', target_yaw: 1.57, target_pitch: 0.5, max_speed_rocket: 30.0}"
```
For more details, please check the code and how can we use this on `README.md`
## About Boat spawn
Est-ce qu'il est possible de modifier l'endroit du spawn des bateaux ?
Actuellement lieu du spawn des équipe de drone est fait aléatoirement dans le "launch_game.sh" script. Est-ce que l'on pourrait intégrer le spawning du bateau à ce script ? Sinon ce script génère ros2_ws/src/px4_pkgs/offboard_control_py/config/spawn_position.yaml avec les position des drones généré par le script. Tu peux aussi te servir de ça comme information sur où placer les bateaux pour chaque équipe.

### Automatic Warship Spawn Location

Regarding the spawn position of the warships, this process has now been fully automated. When you spawn the drones, the warships will automatically appear nearby, positioned relative to the drones’ initial location.

This feature is implemented at **line 148** of the `launch_simulation.sh` file, with the following command:

```bash
python3 $SWARMZ4_PATH/launch_scripts/warship.py $TOTAL_SHIPS 0 0 0 $FIELD_LENGTH $(( FIELD_WIDTH - NUM_DRONES_PER_TEAM + 1 )) 3.14159
```

This ensures the warships are placed appropriately in the simulation, simplifying setup and making the whole initialization process more seamless.

## About game_master link
Faire marcher le canon du bateau avec le missile server.
Dans ros2_ws/src/swarmz_pkgs/game_master/utils/gazebo_subscriber.py, C'est là que je récupère les positions des drones directement dans Gazebo (et pas avec px4) pour calculer leurs distances et alignements pour faire fonctionner missile_sever, kamikaze_server, detections, communications. Je vais chercher le modèles des drones en faisant "px4_prefix+drone_id". Je voudrais faire la même chose avec le canon du bateau "flag_ship_prefix+flag_ship_id", comme ça on pourra directement réutiliser missile_server, detections et communications pour le bateau.

### Integrating the Warship with Game Master Link

Regarding the integration of the warship into the **game_master link**, after reviewing your `missile_server` and `game_master` files, it seems your missile system is already well-implemented and running stably. To make the warship compatible with your system, all I need to do is rename the warship to follow the convention: **flag_ship_prefix + flag_ship_id**.

This is a very straightforward change on my side — I’ll modify the `warship` definition and convert it into a **flag_ship** right away. You can easily verify this change by running `launch_simulation.sh`.

Specifically, this modification simply involves replacing the string `"warship"` with `"flag_ship"` in `warship.py`, which is the file responsible for spawning all ships into the environment based on the launch configuration.

Also, feel free to use my missile firing system if you’d like — I’ve already documented how to use it in the `README.md` file for your convenience.
## About clean code

Est-ce que tu pourrais mettre tous les codes relatifs au bateau dans ros2_ws/src/swarmz_pkgs/flag_ship ? (ou refais toi un nouveau package)
Entre tes codes et ceux de vrx, c'est assez difficle de savoir lesquels il faut modifier.
### Answers
I cann't find some ways to move the `vrx` folder into the `flag_ship` folder. The reason is that the `vrx` folder is contains various package using for bulding the environment and the ship model, this cannot be included on another packages like `flag_ship`

## About vrx & install
Can you make an individual script for the install of vrx ?
And a doc for the new requirements to run gazebo with boats + drones

### Answers
I have built the installation file based on your installation files, specifically as follows:

In the **install_ros2.sh** file, at line 25, I modified it as follows to ensure that your machine has Python 3’s pip installed:
    
    sudo apt update && sudo apt install -y curl python3-pip

Additionally, at line 37 in my file and at line 38 in your file, I installed as follows:

    pip install --user -U empy==3.3.4 pyros-genmsg setuptools==58.2

However, I also noticed this change in your latest commit, so it is not a significant modification.

One of my changes in the installation process is related to PX4 in the **installl_PX4.sh** file. To ensure the installation of Gazebo Garden and the libraries required for ship simulation, I added the following code snippet to the `install_px4` function:

```bash
install_px4() {
    echo "Installing PX4-Autopilot in $SWARMZ4_PATH"

    cd $SWARMZ4_PATH
    git clone --branch v1.15.3 --depth 1 --recurse-submodules https://github.com/PX4/PX4-Autopilot.git
    cd $SWARMZ4_PATH/PX4-Autopilot || { echo "Failed to access PX4 directory"; exit 1; }
    bash ./Tools/setup/ubuntu.sh
    sudo apt install python3-sdformat13 ros-humble-ros-gzgarden ros-humble-xacro -y
    make px4_sitl

    echo "PX4-Autopilot installation completed."
}
```

Furthermore, to ensure that the dependent folders for the boat and flame work properly, in the `copy_world` function I added the following line to ensure that all Gazebo dependency packages can be loaded:

    cp -r $SWARMZ4_PATH/launch_scripts/.simulation-gazebo "$HOME"

This folder is already available in my repository.

These are all my changes based on your source code. You can entirely incorporate these modifications into your installation file to ensure that the boat package runs as smoothly as possible. Regarding the installation of VRX, I inherited your **install_swamz.sh** file and incorporated the above changes into the corresponding files, so my installation is exactly the same as yours. As for the requirements to run the boat and drone, if you add what I mentioned above into your file, I am confident that everything will work normally according to the **readme.md** I wrote.

A small note for you: all my source code works perfectly on ROS2 Humble and Gazebo 7 – also known as Garden.



