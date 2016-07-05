#ifndef KINECT_NETWORK_KINECT_SENDER_H
#define KINECT_NETWORK_KINECT_SENDER_H

#pragma warning(disable:4996)

#include <Kinect.h>

#include <kinect_network/kinect/skeleton.h>

#include <vector>
#include <string>

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
    
    HRESULT initializeSensor();
    void processBody(INT64 time, IBody** bodies);

    // sensor
    IKinectSensor* kinect_sensor_;
    ICoordinateMapper* coordinate_mapper_;

    // body reader
    IBodyFrameReader* body_frame_reader_;

    Skeleton skeletons_[num_bodies_];
};

}

#endif // KINECT_NETWORK_KINECT_SENDER_H