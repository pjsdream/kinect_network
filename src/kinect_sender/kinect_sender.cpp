#include <kinect_network/kinect_sender/kinect_sender.h>
#include <stdio.h>
#include <stdlib.h>

#pragma warning(disable:4996)

// Safe release for interfaces
template<class Interface>
static inline void SafeRelease(Interface *& pInterfaceToRelease)
{
    if (pInterfaceToRelease != NULL)
    {
        pInterfaceToRelease->Release();
        pInterfaceToRelease = NULL;
    }
}


namespace kinect_network
{

KinectSender::KinectSender(const std::string& ip, int port)
    : skeleton_tracking_states_(num_bodies_)
    , skeletons_(num_bodies_)
{
    initializeSensor();
    initializePublisher(ip, port);
}

KinectSender::~KinectSender()
{
    // done with body frame reader
    SafeRelease(body_frame_reader_);

    // done with coordinate mapper
    SafeRelease(coordinate_mapper_);

    // close the Kinect Sensor
    if (kinect_sensor_)
        kinect_sensor_->Close();

    SafeRelease(kinect_sensor_);

    // network
    delete publisher_;
    delete network_context_;
}

HRESULT KinectSender::initializeSensor()
{
    HRESULT hr;

    hr = GetDefaultKinectSensor(&kinect_sensor_);
    if (FAILED(hr))
    {
        return hr;
    }

    if (kinect_sensor_)
    {
        // Initialize the Kinect and get coordinate mapper and the body reader
        IBodyFrameSource* body_frame_source = NULL;

        hr = kinect_sensor_->Open();

        if (SUCCEEDED(hr))
        {
            hr = kinect_sensor_->get_CoordinateMapper(&coordinate_mapper_);
        }

        if (SUCCEEDED(hr))
        {
            hr = kinect_sensor_->get_BodyFrameSource(&body_frame_source);
        }

        if (SUCCEEDED(hr))
        {
            hr = body_frame_source->OpenReader(&body_frame_reader_);
        }

        SafeRelease(body_frame_source);
    }

    if (!kinect_sensor_ || FAILED(hr))
    {
        fprintf(stderr, "No ready Kinect found!");
        return E_FAIL;
    }

    return hr;
}

void KinectSender::initializePublisher(const std::string& ip, int port)
{
    network_context_ = new zmq::context_t(1);
    publisher_ = new zmq::socket_t(*network_context_, ZMQ_PUB);
    
    char address[128];
    sprintf(address, "tcp://%s:%d", ip.c_str(), port);

    publisher_->bind(address);
}

void KinectSender::run()
{
    while (true)
    {
        if (!body_frame_reader_)
        {
            fprintf(stderr, "Body frame reader is not defined\n");
            return;
        }

        IBodyFrame* body_frame = NULL;

        HRESULT hr = body_frame_reader_->AcquireLatestFrame(&body_frame);

        if (SUCCEEDED(hr))
        {
            INT64 time = 0;

            hr = body_frame->get_RelativeTime(&time);

            IBody* bodies[num_bodies_] = {0};

            if (SUCCEEDED(hr))
            {
                hr = body_frame->GetAndRefreshBodyData(_countof(bodies), bodies);
            }

            if (SUCCEEDED(hr))
            {
                processBody(time, bodies);

                // send skeleton through network
                sendSkeletons();
            }

            for (int i = 0; i < _countof(bodies); ++i)
            {
                SafeRelease(bodies[i]);
            }
        }

        SafeRelease(body_frame);
    }
}

void KinectSender::processBody(INT64 time, IBody** bodies)
{
    HRESULT hr;

    for (int i = 0; i < num_bodies_; ++i)
    {
        IBody* body = bodies[i];
        if (body)
        {
            BOOLEAN tracked = false;
            hr = body->get_IsTracked(&tracked);

            if (SUCCEEDED(hr) && tracked)
            {
                skeleton_tracking_states_[i] = true;

                Joint joints[JointType_Count]; 
                HandState left_hand_state = HandState_Unknown;
                HandState right_hand_state = HandState_Unknown;

                body->get_HandLeftState(&left_hand_state);
                body->get_HandRightState(&right_hand_state);

                hr = body->GetJoints(_countof(joints), joints);

                for (int j=0; j<Skeleton::numJoints(); j++)
                {
                    Joint& joint = joints[j];

                    const int joint_index = joint.JointType;

                    Eigen::Vector3f position(joint.Position.X, joint.Position.Y, joint.Position.Z);

                    Skeleton::JointState joint_state;
                    switch (joint.TrackingState)
                    {
                    case TrackingState_NotTracked:
                        joint_state = Skeleton::JointState::JointNotTracked;
                        break;

                    case TrackingState_Inferred:
                        joint_state = Skeleton::JointState::JointInferred;
                        break;

                    case TrackingState_Tracked:
                        joint_state = Skeleton::JointState::JointTracked;
                        break;
                    }

                    skeletons_[i].setJointPosition(j, position);
                    skeletons_[i].setJointState(j, joint_state);
                }
            }
            else
            {
                skeleton_tracking_states_[i] = false;
            }
        }
    }
}

void KinectSender::sendSkeletons()
{
    zmq::message_t msg(sizeof(int) + sizeof(Skeleton));

    for (int i=0; i<num_bodies_; i++)
    {
        if (skeleton_tracking_states_[i])
        {
            // first 4 byte field is the body id
            *(int*)msg.data() = i;

            // next, skeleton data follows
            memcpy((int *)msg.data() + 1, &skeletons_[i], sizeof(Skeleton));

            publisher_->send(msg);
            printf("Sent body %d\n", i);
        }
    }
}

}
