# Deployment of Allegro Hand

## Disclaimer

This repository and README is heavily based on Haozhi's Allegro ROS repository (https://github.com/HaozhiQi/ros-allegro.git)

## Installation

### Socket CAN Setup
  #### Set SocketCAN bitrate
  ```
  sudo ip link set can0 type can bitrate 1000000
  sudo ip link set can0 up
  ```

  #### [Optional] disable previously installed peakcan driver
  ```
  # Temporarily
  sudo rmmod pcan
  sudo modprobe peak_usb

  # Permanent
  rm /etc/modprobe.d/pcan.conf
  rm /etc/modprobe.d/blacklist-peak.conf
  ```


### Install from conda
```
mamba install -c tingfan -c robostack-staging ros-humble-allegro-hand-controllers
```

### Install this repository 

```
git clone -b ros2-main https://github.com/0wu/ros-allegro.git
colcon build
source install/setup.bash
```

### Launch
Use following command to launch ROS node, if a serial number is reported, your hardware+driver is working.
```
ros2 run allegro_hand_controllers allegro_node_pd

[... deleted ...]

*************************************
      Joint PD Control Method        
-------------------------------------
  Only 'H', 'O', 'S', 'Space' works. 
*************************************
[INFO] [1695759425.789599207] [allegro_hand_core_pd]: Polling = false.
>CAN(0): AllegroHand hardware version: 0x0402
                      firmware version: 0x0405
                      hardware type: 0(right)
                      temperature: 54 (celsius)
                      status: 0x08
                      servo status: OFF
                      high temperature fault: OFF
                      internal communication fault: OFF
>CAN(0): AllegroHand serial number: SAH040 091R
```

### Parameters
#### Dump parameters from code
```
# launch the node in simulation mode, so no physical CAN device is needed
$ ros2 run allegro_hand_controllers allegro_node_pd --sim
$ ros2 param dump /allegro_node_pd > allegro_hand_params.yaml
```

#### Launch with Parameters
```
$ ros2 run allegro_hand_controllers allegro_node_pd --ros-args --params-file allegro_hand_params.yaml
```

### Instructions

Can use `ros2 topic list` to monitor the ROS topics.

```shell
$ ros2 topic list
/allegroHand/joint_cmd
/allegroHand/joint_states
/allegroHand/lib_cmd
```

### Python Client
```
from allegro_hand_controllers.allegro_robot import AllegroRobot
import threading

# setup rosnode
rclpy.init()
hand = AllegroRobot(hand_topic_prefix='/allegroHand')
threading.Thread(target=rclpy.spin, args=[hand]).start()

# poll joint position and command
init_pos, init_vel = hand.poll_joint_position(wait=True)
hand.command_joint_position(pos)
hand.disconnect() # poweroff
```

# BUILD Conda package
* reference https://medium.com/robostack/cross-platform-conda-packages-for-ros-fa1974fd1de3

```
# work dir
mkdir conda_build; cd conda_build;

# generate conda recipe with vinca (or link the recipe.yaml provided in repo root)
# pip install git+https://github.com/RoboStack/vinca.git
vinca -p '../*/*/package.xml' -d ..
cp ../recipe.yaml .


# setup additional channels
conda config --add channels conda-forge
conda config --add channels robostack-staging
conda config --add channels tingfan

# build conda pacakge
mamba install -c conda-forge boa
# rm -rf $CONDA_PREFIX/conda-bld  # remove old conda bld repo if nee
boa build ..

# check built packages
ls $CONDA_PREFIX/conda-bld/linux-64/*tar.bz2

# upload to conda-forge
mamba install -c conda-forge anaconda-client
anaconda upload $CONDA_PREFIX/conda-bld/linux-64/*tar.bz2
```
