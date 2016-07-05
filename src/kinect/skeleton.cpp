#include <kinect_network/kinect/skeleton.h>


namespace kinect_network
{

const std::vector<std::string> Skeleton::joint_names_ = 
{
    "spine_base",
    "spine_mid",
    "neck",
    "head",
    "shoulder_left",
    "elbow_left",
    "wrist_left",
    "hand_left",
    "shoulder_right",
    "elbow_right",
    "wrist_right",
    "hand_right",
    "hip_left",
    "knee_left",
    "ankle_left",
    "foot_left",
    "hip_right",
    "knee_right",
    "ankle_right",
    "foot_right",
    "spine_shoulder",
    "hand_tip_left",
    "thumb_left",
    "hand_tip_right",
    "thumb_right",
};

Skeleton::Skeleton()
    : left_hand_state_(HandStateUnknown)
    , right_hand_state_(HandStateUnknown)
{
    for (int i=0; i<num_joints_; i++)
        joint_states_[i] = JointNotTracked;
}

void Skeleton::setJointPosition(int joint_index, const Eigen::Vector3f& position)
{
    joint_positions_[joint_index] = position;
}

void Skeleton::setJointState(int joint_index, JointState joint_state)
{
    joint_states_[joint_index] = joint_state;
}

void Skeleton::printSkeleton() const
{
    for (int i=0; i<num_joints_; i++)
    {
        printf("%14s: %10.6f %10.6f %10.6f ", joint_names_[i].c_str(), joint_positions_[i].x(), joint_positions_[i].y(), joint_positions_[i].z());

        switch (joint_states_[i])
        {
        case JointNotTracked:
            printf("(Not tracked)\n");
            break;

        case JointInferred:
            printf("(Inferred)\n");
            break;

        case JointTracked:
            printf("(Tracked)\n");
            break;

        default:
            printf("(Undefined state)\n");
            break;
        }
    }
}

}
