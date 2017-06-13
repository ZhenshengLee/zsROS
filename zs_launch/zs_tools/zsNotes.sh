sudo apt-get install libsdl1.2-dev libsdl-image1.2-dev ros-indigo-navigation ros-indigo-axis-camera ros-indigo-flir-ptu-driver ros-indigo-serial
sudo apt-get install ros-indigo-freenect-stack ros-indigo-openni* libfreenect-dev
sudo apt install ros-indigo-joy-listener ros-indigo-joy-teleop ros-indigo-teleop-tools ros-indigo-teleop-twist-joy ros-indigo-teleop-twist-keyboard
sudo apt install ros-indigo-lms1xx
sudo apt install ros-indigo-rosbridge-suite
# for kinetic
sudo apt install ros-kinetic-navigation
sudo apt install ros-kinetic-freenect-stack ros-kinetic-openni* libfreenect-dev ros-kinetic-serial
sudo apt install ros-kinetic-joy-listener ros-kinetic-joy-teleop ros-kinetic-teleop-tools ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard
sudo apt install ros-kinetic-lms1xx
# This is for kinectv1, for kinectv2 please refer to github libfreenect2
sudo apt install ros-kinetic-rosbridge-suite
# set ip config in network manager
# Address	192.168.0.4
# Netmask	192.0.255.255(automatically become 192.0.0.0, could work but I don't know why)
# Gateway	192.168.0.1

sudo chmod a+rwx /dev/ttyS0

sudo chmod a+rwx /dev/ttyS0
roscore
rqt_console
rqt_graph


sudo chmod a+rwx /dev/ttyS0

# for simulator
roscore
roscd zs_launch_files/
./zs_tools.sh cmee
roslaunch zs_launch_files zs_p3at_rviz_full.launch

# for experiment
# In P3AT
# 看看能不能弄一个launch文件，机器会不会报错
roscore

sudo chmod a+rwx /dev/ttyS0
roslaunch zs_launch_files zs_rosaria_ex.launch

roslaunch zs_launch_files zs_proxy.launch

roslaunch zs_launch_files zs_client.launch

roslaunch zs_launch_files zs_move_base_ex.launch

# In third party computer

roslaunch zs_launch_files zs_p3at_rviz.launch

# multiple machine simulation
# Please set IP before everything OK
# Test your Proxy

# better launch
roscore

roscd zs_launch_files/
./zs_tools.sh cmee

roslaunch zs_launch_files zs_rosaria.launch

roslaunch zs_launch_files zs_proxy.launch

roslaunch zs_launch_files zs_move_base.launch

roslaunch zs_launch_files zs_client.launch

# launch in experiment in real robot!!


# for experiment 170425
roscore

sudo chmod a+rwx /dev/ttyS0
roslaunch zs_launch_files zs_rosaria_ex.launch

roslaunch zs_launch_files zs_joy_rc.launch

# roslaunch zs_launch_files zs_client.launch

roslaunch zs_launch_files zs_move_base_ex.launch

# In third party computer

roslaunch zs_launch_files zs_p3at_rviz.launch

# multiple machine simulation
# Please set IP before everything OK
# Test your Proxy

# better launch

# 看看能不能弄一个launch文件，机器会不会报错
roscore

sudo chmod a+rwx /dev/ttyS0
roslaunch zs_launch_files zs_rosaria_ex.launch

roslaunch zs_launch_files zs_joy_rc.launch

# roslaunch zs_launch_files zs_client.launch

roslaunch zs_launch_files zs_move_base_ex.launch

# In third party computer

roslaunch zs_launch_files zs_p3at_rviz_ex.launch