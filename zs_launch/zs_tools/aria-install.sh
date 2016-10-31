# do not use rosdep to auto install, let it be installed from source

# To get the rosaria package to compile, here is what i did:

# I assume that you installed ROS using sudo apt install ros-kinetic-desktop-full
# I then downloaded the libaria_2.9.0+ubuntu12+gcc4.6_amd64.deb installation file from here
# Installed the .deb file with sudo dpkg -i libaria_2.9.0+ubuntu12+gcc4.6_amd64.deb
# went to /usr/local/Aria and ran make clean; make
# went to my catkin workspace and built the package with catkin build.
# I was then able to source my workspace, and run the node with rosrun rosaria RosAria. I am unfortunately not able to test it, as I am quite far from my robot at the moment.