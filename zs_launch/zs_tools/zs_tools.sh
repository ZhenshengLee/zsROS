echo "Welcome! zs"
echo "Let's enjoy with some useful tools!"
echo "Sourcing .bashrc file"
source ~/.bashrc
echo "Launching MobileSim with the default map"
/usr/local/MobileSim/MobileSim -m $PWD/zs_tools/columbia.map -r p3at &
echo "Launching rqt_tools"
rqt_graph &
rqt_console &
rqt &
echo "Launching RViz"
roslaunch zs_launch_files zs_p3at_rviz.launch
echo "Finishing!"