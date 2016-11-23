sudo apt-get install libsdl1.2-dev libsdl-image1.2-dev ros-indigo-move-base

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
