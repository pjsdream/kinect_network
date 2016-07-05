#include <kinect_network/kinect_receiver/kinect_receiver.h>

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        fprintf(stderr, "Usage: kinect_receiver IP_ADDR PORT\n");
        return 1;
    }

    kinect_network::KinectReceiver receiver(argv[1], atoi(argv[2]));

    receiver.run();

    return 0;
}
