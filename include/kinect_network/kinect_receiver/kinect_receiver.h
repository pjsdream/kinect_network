#ifndef KINECT_NETWORK_KINECT_RECEIVER_H
#define KINECT_NETWORK_KINECT_RECEIVER_H

// to remove redefinition conflict with winsock.h
#include <WinSock2.h>

#include <kinect_network/kinect/skeleton.h>
#include <zmq.hpp>

namespace kinect_network
{

class KinectReceiver
{
private:

    static const int num_bodies_ = 6;

public:

    KinectReceiver();
    ~KinectReceiver();

    void run();

private:
    
    // network
    void initializeSubscriber();

    // skeleton data
    std::vector<char> skeleton_tracking_states_;
    std::vector<Skeleton> skeletons_;

    // network
    zmq::context_t* network_context_;
    zmq::socket_t* subscriber_;
};

}

#endif // KINECT_NETWORK_KINECT_RECIEVER_H