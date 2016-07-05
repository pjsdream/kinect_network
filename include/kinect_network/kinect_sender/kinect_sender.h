#ifndef KINECT_NETWORK_KINECT_SENDER_H
#define KINECT_NETWORK_KINECT_SENDER_H

// to remove redefinition conflict with winsock.h
#include <WinSock2.h>

#include <Kinect.h>
#include <kinect_network/kinect/skeleton.h>
#include <zmq.hpp>

namespace kinect_network
{

class KinectSender
{
private:

    static const int num_bodies_ = BODY_COUNT;

public:

    KinectSender();
    ~KinectSender();

    void run();

private:
    
    // kinect
    HRESULT initializeSensor();
    void processBody(INT64 time, IBody** bodies);

    void sendSkeletons();

    // network
    void initializePublisher();

    // sensor
    IKinectSensor* kinect_sensor_;
    ICoordinateMapper* coordinate_mapper_;

    // body reader
    IBodyFrameReader* body_frame_reader_;

    std::vector<char> skeleton_tracking_states_;
    std::vector<Skeleton> skeletons_;

    // network
    zmq::context_t* network_context_;
    zmq::socket_t* publisher_;
};

}

#endif // KINECT_NETWORK_KINECT_SENDER_H