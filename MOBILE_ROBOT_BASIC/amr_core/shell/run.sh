sudo slcand -o -c -s6 /dev/ttyACM* can0
sudo ifconfig can0 up
roslaunch motor114_core core.launch