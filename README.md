# UAV_Commander
kernel command software for UAV

this is "develop" branch of UCD-UAV repo. only working code will be pushed to this repo.
Because keep the directory organization consistent

## Running the Code
Please follow closely to the following steps.

We need to first install catkin_tools to allow easy configuration of the default catkin source space. Please execute the following commands. (Note: this only works under ubuntu since we are using apt-get, for installations under different platform/linux distro, please consult 'http://catkin-tools.readthedocs.org/en/latest/installing.html').
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install python-catkin-tools
```
.

After installation of the catkin-tools, under project directory of your choice (we will call it DIR for now as a placeholder; further more, we assume that you have put the UAV_Commander within DIR), execute the following command:

```bash
cd DIR
source /opt/ros/jade/setup.bash
catkin config -s DIR/UAV_Commander
catkin build
```

Note, we are not using catkin_make because it would be default the source space to DIR/src. This is undesirable.

Now, we source the built environment:

```bash
source devel/setup.sh
```
.

We can verify the successful building of the project by seeing if our custom message is included in the ROS package:

```bash
rosmsg list | grep dart
```

### Relaying Message to GCS
In order to relay message back to the Ground Control Station, we will need to use mavros.mavlink message. This will require the pymavlink module. Please consult'https://pixhawk.org/dev/pymavlink' for installation instructions.
.
