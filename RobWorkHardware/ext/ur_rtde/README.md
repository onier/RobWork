# Universal Robots RTDE C++ Interface #
A C++ interface for sending and receiving data to/from a UR robot using the 
[Real-Time Data Exchange (RTDE)](https://www.universal-robots.com/how-tos-and-faqs/how-to/ur-how-tos/real-time-data-exchange-rtde-guide-22229/)
 interface of the robot. 
 
### Documentation ###
Documentation with examples and API resides at <https://caro-sdu.gitlab.io/ur_rtde/>

### Build Status on Ubuntu 16.04 (xenial) ###
[![build status](https://gitlab.com/caro-sdu/ur_rtde/badges/master/build.svg)](https://gitlab.com/caro-sdu/ur_rtde/commits/master)

### Dependencies ###
*  Boost

### Compatible Robots ###

*  All CB-Series from CB3/CB3.1 software 3.3
*  All e-Series

### Compatible Operating Systems ###
Currently tested on:

*  Ubuntu 16.04 (Xenial Xerus)
*  Ubuntu 18.04 (Bionic Beaver)

### Build Instructions ###
    mkdir build
    cd build
    cmake ..
    make
    
### Test ###
In order to test the interface, download the most recent UR simulator for your robot from here [UR Download](https://www.universal-robots.com/download/). Once installed
run the simulator with:

    ./start-ursim.sh

When the simulator is running initialize the robot by first clicking the **'ON'** button, and next clicking the **'START'** button. You can now run the examples.

### Contact ###
If you have any questions or suggestions to the interface, feel free to contact <anpl@mmmi.sdu.dk>
