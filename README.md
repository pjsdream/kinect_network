# kinect_network
My personal project for communicating betwee Windows and Linux

## kinect_receiver.launch

* Subscribed topics
** none

* Published topics
** 25 tf's suffixed by body id (e.g., spine_base_0) ranging from 0 to 5 (inclusive):  
    "spine_base",  
    "spine_mid",  
    "neck",  
    "head",  
    "shoulder_left",  
    "elbow_left",  
    "wrist_left",  
    "hand_left",  
    "shoulder_right",  
    "elbow_right",  
    "wrist_right",  
    "hand_right",  
    "hip_left",  
    "knee_left",  
    "ankle_left",  
    "foot_left",  
    "hip_right",  
    "knee_right",  
    "ankle_right",  
    "foot_right",  
    "spine_shoulder",  
    "hand_tip_left",  
    "thumb_left",  
    "hand_tip_right",  
    "thumb_right",  
** skeleton_visualizer/skeleton (type: visualization_msgs)


## Build kinect_sender on Windows connected to Kinect

1. git clone
2. Install Visual Studio 2015
3. Install Kinect v2 SDK (https://www.microsoft.com/en-us/download/details.aspx?id=44561)
4. Download Eigen (http://eigen.tuxfamily.org/index.php?title=Main_Page) and put the folder to include/
5. Download libzmq (http://zeromq.org/intro:get-the-software) 4.1.5 (Windows sources recommended), copy zmq.h and zmq_utils.h to include/, copy libzmq.lib to lib/, copy libzmq.dll to bin/ if dynamic libraries generated. May need to download libsodium to build libzmq
6. Download libzmq C++ bindings (http://zeromq.org/bindings:cpp) i.e., 2 header files, and copy them to include/
7. Download libzmq helper header file (https://github.com/imatix/zguide/blob/master/examples/C/zhelpers.h) and copy it to include/
8. Open msvc solution file (projects/kinect_network.sln) and build all

## Run kinect_sender on Windows

$ kinect_sender IP_ADDR PORT

## Build kinect_receiver with ROS on Linux

1. git clone
2. Download libzmq (http://zeromq.org/intro:get-the-software) 4.1.5 (Linux sources recommended) and copy zmq.h and zmq_utils.h to include/.
3. Download libzmq C++ bindings (http://zeromq.org/bindings:cpp) i.e., 2 header files, and copy them to include/
4. catkin_make --pkg kinect_network

## Run kinect_receiver

$ kinect_receiver IP_ADDR PORT  
or  
$ rosrun kinect_network kinect_receiver IP_ADDR PORT

## Examples

On Windows:  
$ kinect_sender (sender's ip) 5556

On Linux:  
$ kinect_receiver (sender's ip) 5556  
or  
$ rosrun kinect_network kinect_receiver (sender's ip) 5556

## Launch

$ roslaunch vi
