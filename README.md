#UAV Control

##Setup

    cd ~/catkin_ws/src
    git clone https://github.com/mickey13/uav-control.git uav_control
    cd ~/catkin_ws/
    catkin_make

##Run Control Program

    source devel/setup.bash
    rosrun uav_control teleop_keyboard.py

