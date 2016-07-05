#ifndef KINECT_NETWORK_ROS_KINECT_RECEIVER_H
#define KINECT_NETWORK_ROS_KINECT_RECEIVER_H

#include <kinect_network/kinect_receiver/kinect_receiver.h>
#include <tf/transform_broadcaster.h>

namespace kinect_network
{

class RosKinectReceiver : public KinectReceiver
{
public:

    RosKinectReceiver(const std::string& ip, int port, const ros::NodeHandle& node_handle = ros::NodeHandle("~"));
    ~RosKinectReceiver();

    virtual void run();

private:

    ros::NodeHandle node_handle_;
    tf::TransformBroadcaster broadcaster_;
};

}

#endif // KINECT_NETWORK_KINECT_RECIEVER_H
