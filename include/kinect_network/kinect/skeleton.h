#ifndef KINECT_NETWORK_SKELETON_H
#define KINECT_NETWORK_SKELETON_H

#include <Kinect.h>

#include <vector>
#include <string>
#include <Eigen/Dense>

namespace kinect_network
{

class Skeleton
{
private:

    static const int num_joints_ = JointType_Count;
    static const std::vector<std::string> joint_names_;

public:

    static inline int numJoints()
    {
        return num_joints_;
    }

    enum JointState
    {
        JointNotTracked = 0,
        JointInferred = 1,
        JointTracked = 2,
    };

public:

    Skeleton();

    void setJointPosition(int joint_index, const Eigen::Vector3f& joint_position);
    void setJointState(int joint_index, JointState joint_state);

    void printSkeleton();

private:

    std::vector<Eigen::Vector3f> joint_positions_;
    std::vector<JointState> joint_states_;
};

}

#endif // KINECT_NETWORK_SKELETON_H