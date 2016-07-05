#include <kinect_network/kinect_receiver/kinect_receiver.h>
#include <stdio.h>
#include <stdlib.h>


namespace kinect_network
{

KinectReceiver::KinectReceiver()
    : skeleton_tracking_states_(num_bodies_)
    , skeletons_(num_bodies_)
{
    initializeSubscriber();
}

KinectReceiver::~KinectReceiver()
{
    // network
    delete subscriber_;
    delete network_context_;
}

void KinectReceiver::initializeSubscriber()
{
    network_context_ = new zmq::context_t(1);
    subscriber_ = new zmq::socket_t(*network_context_, ZMQ_SUB);
    subscriber_->connect("tcp://localhost:5556");
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
