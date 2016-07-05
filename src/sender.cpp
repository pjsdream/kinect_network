#include <kinect_network/kinect_sender/kinect_sender.h>

int main(int argc, char** argv)
{
    if (argc != 3)
    {
        fprintf(stderr, "Usage: kinect_sender IP_ADDR PORT\n");
        return 1;
    }

    kinect_network::KinectSender sender(argv[1], atoi(argv[2]));

    sender.run();

    return 0;
}
