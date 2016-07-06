#ifndef KINECT_NETWORK_SKELETON_H
#define KINECT_NETWORK_SKELETON_H

#include <vector>
#include <string>
#include <Eigen/Dense>

namespace kinect_network
{

class Skeleton
{
private:

    static const int num_joints_ = 25;
    static const std::vector<std::string> joint_names_;

public:

    static inline int getNumJoints()
    {
        return num_joints_;
    }

    static const std::vector<std::string>& getJointNames()
    {
        return joint_names_;
    }

    enum JointState
    {
        JointNotTracked = 0,
        JointInferred = 1,
        JointTracked = 2,
    };
    
    enum HandState
    {
        HandStateUnknown = 0,
        HandStateNotTracked = 1,
        HandStateOpen = 2,
        HandStateClosed = 3,
        HandStateLasso = 4,
    };

public:

    Skeleton();

    void setJointPosition(int joint_index, const Eigen::Vector3f& joint_position);
    void setJointState(int joint_index, JointState joint_state);
    
    inline void setLeftHandState(HandState hand_state)
    {
        left_hand_state_ = hand_state;
    }

    inline void setRightHandState(HandState hand_state)
    {
        right_hand_state_ = hand_state;
    }

    inline const Eigen::Vector3f& getJointPosition(int joint_index) const
    {
        return joint_positions_[joint_index];
    }

    inline const std::string& getJointName(int joint_index) const
    {
        return joint_names_[joint_index];
    }

    void printSkeleton() const;

private:

    Eigen::Vector3f joint_positions_[num_joints_];
    JointState joint_states_[num_joints_];

    HandState left_hand_state_;
    HandState right_hand_state_;
};

}

#endif // KINECT_NETWORK_SKELETON_H
