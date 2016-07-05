#include <kinect_network/kinect_receiver/ros_kinect_receiver.h>
#include <stdio.h>
#include <stdlib.h>
#include <eigen_conversions/eigen_msg.h>


namespace kinect_network
{

RosKinectReceiver::RosKinectReceiver(const std::string& ip, int port, const ros::NodeHandle& node_handle)
    : KinectReceiver(ip, port)
    , node_handle_(node_handle)
{
}

RosKinectReceiver::~RosKinectReceiver()
{
}

void RosKinectReceiver::run()
{
    zmq::message_t msg;

    while (true)
    {
        subscriber_->recv(&msg);

        const int id = *(int *)msg.data();
        const Skeleton* skeleton = (Skeleton *)((int *)msg.data() + 1);

        printf("Body %d recieved\n", id);
        skeleton->printSkeleton();

        for (int i=0; i<Skeleton::numJoints(); i++)
        {
            const std::string& joint_name = skeleton->getJointName(i);
            const Eigen::Vector3f joint_position = skeleton->getJointPosition(i);

            tf::Transform transform;
            transform.setIdentity();
            transform.setOrigin( tf::Vector3(joint_position.x(), joint_position.y(), joint_position.z()) );
            broadcaster_.sendTransform( tf::StampedTransform(transform, ros::Time::now(), "kinect_depth_frame", joint_name) );
        }
    }
}

}
