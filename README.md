## Setting up the server_pkg for testing

### Step 1) Make sure ROS Melodic or higher is installed on your machine
### Step 2) Download the catkin command line tools (make sure you have pip3 first)

```sudo pip3 install -U catkin_tools```

### Step 2) Setup a catkin workspace for holding the package

```mkdir -p ~/catkin_ws/src```
```cd ~/catkin_ws/```
```catkin build```
### Step 3) once the workspace is initially built add the necessary packages from github
```cd ~/catkin_ws/src```
```git clone https://github.com/eve-Vhx/msg_pkg.git```
```git clone https://github.com/eve-Vhx/server_pkg_testing.git```
### Step 4) Recompile workspace
```cd ~/catkin_ws```
```catkin_build```
### Step 5) Source the workspace (This allows ros to use all the necessary tools to run the scripts'
```source devel/setup.bash```
### Step 6) Make all python files executable
```cd ~/catkin_ws/src/server_pkg_testing/src```
```chmod +x .```
### Step 7) Launch the launch files
```roslaunch server_pkg_testing fake_pi_data.launch```
```roslaunch server_pkg_testing server_master.launch```
### Step 8) Make sure the data is flowing
```rostopic echo /QROW11021/ui_telem_data```

