# kinect_network
My personal project for communicating betwee Windows and Linux

## Build kinect_sender on Windows connected to Kinect

1. git clone
2. Install Visual Studio 2015
3. Install Kinect v2 SDK (https://www.microsoft.com/en-us/download/details.aspx?id=44561)
4. Download Eigen (http://eigen.tuxfamily.org/index.php?title=Main_Page) and put the folder to include/
5. Download libzmq (http://zeromq.org/intro:get-the-software) 4.1.5 (Windows sources recommended), copy zmq.h and zmq_utils.h to include/, copy libzmq.lib to lib/, copy libzmq.dll to bin/ if dynamic libraries generated. May need to download libsodium to build libzmq
6. Download libzmq C++ bindings (http://zeromq.org/bindings:cpp) i.e., 2 header files, and copy them to include/
7. Download libzmq helper header file (https://github.com/imatix/zguide/blob/master/examples/C/zhelpers.h) and copy it to include/
8. Open msvc solution file (projects/kinect_network.sln) and build all

## Build kinect_receiver on Linux

1. git clone

## Run kinect_sender

$ kinect_sender IP_ADDR PORT

## Run kinect_receiver

$ kinect_receiver IP_ADDR PORT

## Examples

$ kinect_sender 127.0.0.1 5556  
$ kinect_receiver 127.0.0.1 5556