echo "Welcome! zs"
echo "Let's enjoy with some useful tools!"
# echo "Setup the ROS_IP as 10.0.126.3"
# echo "export ROS_IP=10.0.126.3" >> ~/.bashrc
echo "Sourcing .bashrc file"
source ~/.bashrc
echo "Launching MobileSim with the default map"
# /usr/local/MobileSim/MobileSim -m $PWD/zs_tools/columbia.map -r p3at &
/usr/local/MobileSim/MobileSim -m $PWD/zs_tools/cmee.map -r p3at &
echo "Launching rqt_tools"
rqt_graph &
rqt_console &
# zs: MUST RUN roscore HERE; DO NOT RUN MASTER AUTOMATICALLY
# echo "Launching roscore"
# roscore &
echo "Launching RViz"
roslaunch zs_launch_files zs_p3at_rviz.launch
echo "Finishing!"

# roslaunch zs_launch_files zs_rosaria.launch
# roslaunch zs_launch_files zs_move_base.launch