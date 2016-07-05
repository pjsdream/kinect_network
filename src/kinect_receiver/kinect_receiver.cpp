#include <kinect_network/kinect_receiver/kinect_receiver.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef _WIN32
#pragma warning(disable:4996)
#endif

namespace kinect_network
{

KinectReceiver::KinectReceiver(const std::string& ip, int port)
    : skeleton_tracking_states_(num_bodies_)
    , skeletons_(num_bodies_)
{
    initializeSubscriber(ip, port);
}

KinectReceiver::~KinectReceiver()
{
    // network
    delete subscriber_;
    delete network_context_;
}

void KinectReceiver::initializeSubscriber(const std::string& ip, int port)
{
    network_context_ = new zmq::context_t(1);
    subscriber_ = new zmq::socket_t(*network_context_, ZMQ_SUB);

    char address[128];
    sprintf(address, "tcp://%s:%d", ip.c_str(), port);

    subscriber_->connect(address);
    subscriber_->setsockopt(ZMQ_SUBSCRIBE, "", 0);
}

void KinectReceiver::run()
{
    zmq::message_t msg;

    while (true)
    {
        subscriber_->recv(&msg);

        const int id = *(int *)msg.data();
        const Skeleton* skeleton = (Skeleton *)((int *)msg.data() + 1);

        printf("Body %d recieved\n", id);
        skeleton->printSkeleton();
    }
}

}
