#include <kinect_network/kinect_receiver/ros_kinect_receiver.h>

#include <ros/ros.h>

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        fprintf(stderr, "Usage: kinect_receiver IP_ADDR PORT\n");
        return 1;
    }

    ros::init(argc, argv, "ros_kinect_receiver");

    kinect_network::RosKinectReceiver receiver(argv[1], atoi(argv[2]));

    receiver.run();

    return 0;
}
