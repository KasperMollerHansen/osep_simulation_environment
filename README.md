# PX4-Autopilot in simulation envirmonet using ROS2

Virtuel enviroments can in general be an issue, so it's recomended to deactivate.
```
conda deactivate
```

<details>
<summary>Guide to install Isaac Sim 4.2.0 </summary>

This guide is slightly modified from:
<https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_workstation.html>

### Install Isaac Sim
Ensure system requirements as stated by the documentation...

Download Isaac Sim 4.2.0 from <https://docs.isaacsim.omniverse.nvidia.com/latest/installation/download.html#isaac-sim-latest-release> and place in Downloads

```
mkdir ~/isaacsim
cd ~/Downloads
unzip "your-downloaded-file.zip" -d ~/isaacsim
cd ~/isaacsim
./omni.isaac.sim.post.install.run.sh
./isaac-sim.sh
```
Isaac Sim should now open

Create alias in ~/.bashrc to use the scripts in this repo.
```
alias ISAAC_ENV="source ~/isaacsim/setup_conda_env.sh"
```

</details>

<details>
<summary>Guide to install Pegasus Simulator for Isaac Sim 4.2.0 </summary>

This guide is slightly modified from:
<https://pegasussimulator.github.io/PegasusSimulator/>

### Install Pegasus Simulator
```
ISAAC_ENV
cd
git clone https://github.com/PegasusSimulator/PegasusSimulator.git
cd PegasusSimulator/extensions
python -m pip install --editable pegasus.simulator
```

</details>

<details>
<summary>Guide to install PX4-Autopilot </summary>

This guide is sligthly modified from:
<https://docs.px4.io/main/en/ros2/user_guide.html#install-px4>

### Install PX4
```
cd
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
cd PX4-Autopilot/
make px4_sitl
```
### Install ROS2

```
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt upgrade -y
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> .bashrc
```
### Install the Micro XRCE-DDS Agent
```
python3 -m pip install --upgrade pip
python3 -m pip install --upgrade setuptools wheel twine check-wheel-contents
```
The PX4 uses version v2.x.x
```
git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```
### Get the ROS2 msg for PX4
```
mkdir -p ~/ws_ros2/src/
cd ~/ws_ros2/src/
git clone https://github.com/PX4/px4_msgs.git
cd ..
colcon build
```
#### Remember to source the ROS2 WS
```
echo source ~/ws_ros2/install/setup.bash >> ~/.bashrc 
```

</details>

<details>
<summary>Guide to install the pegasus_sim_env pkg </summary>

### Clone the pkg ###
```
mkdir -p ~/ws_ros2/src/
cd ~/ws_ros2/src/
git clone https://github.com/KasperMollerHansen/pegasus_sim_env.git
cd ..
colcon build
```

### Setup Alias ###
Create alias in ~/.bashrc to use the scripts in this repo.
```
alias pegasus_launch="cd ~/ws_ros2 && source install/setup.bash && ./src/pegasus_sim_env/launch_pegasus.sh"
```
</details>


## Running simulation

This repository contains the simulation setup along with a launch file that is used together with Nvblox.

To run the Isaac Sim simulation
```
pegasus_launch
```
To launch the planner used with Nvblox
```
source install/setup.bash
ros2 launch pegasus_sim_env pegasus.launch.py
```
To run the testflight
```
ISAAC_ENV
source install/setup.bash
ros2 run pegasus_sim_env test_flight.py
```
