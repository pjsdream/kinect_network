#ifndef KINECT_NETWORK_KINECT_RECEIVER_H
#define KINECT_NETWORK_KINECT_RECEIVER_H

// to remove redefinition conflict with winsock.h
#ifdef _WIN32
#include <WinSock2.h>
#endif

#include <kinect_network/kinect/skeleton.h>
#include <zmq.hpp>

namespace kinect_network
{

class KinectReceiver
{
private:

    static const int num_bodies_ = 6;

public:

    KinectReceiver(const std::string& ip, int port);
    ~KinectReceiver();

    void run();

private:
    
    // network
    void initializeSubscriber(const std::string& ip, int port);

    // skeleton data
    std::vector<char> skeleton_tracking_states_;
    std::vector<Skeleton> skeletons_;

    // network
    zmq::context_t* network_context_;
    zmq::socket_t* subscriber_;
};

}

#endif // KINECT_NETWORK_KINECT_RECIEVER_H
