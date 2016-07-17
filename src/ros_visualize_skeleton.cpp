#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <kinect_network/kinect/skeleton.h>
#include <eigen_conversions/eigen_msg.h>

const std::vector<std::pair<int, int> > edges =
{
    { 0,  1},
    { 1, 20},
    {20,  2},
    { 2,  3},
    {20,  4},
    { 4,  5},
    { 5,  6},
    { 6,  7},
    {20,  8},
    { 8,  9},
    { 9, 10},
    {10, 11},
    { 0, 12},
    {12, 13},
    {13, 14},
    {14, 15},
    { 0, 16},
    {16, 17},
    {17, 18},
    {18, 19},
    { 7, 21},
    { 7, 22},
    {11, 23},
    {11, 24},
};

const double radius = 0.05;

/*
0   "spine_base",
1   "spine_mid",
2   "neck",
3   "head",
4   "shoulder_left",
5   "elbow_left",
6   "wrist_left",
7   "hand_left",
8   "shoulder_right",
9   "elbow_right",
0   "wrist_right",
1   "hand_right",
2   "hip_left",
3   "knee_left",
4   "ankle_left",
5   "foot_left",
6   "hip_right",
7   "knee_right",
8   "ankle_right",
9   "foot_right",
0   "spine_shoulder",
1   "hand_tip_left",
2   "thumb_left",
3   "hand_tip_right",
4   "thumb_right",
*/

void visualizeSkeleton(int body_id, const std::vector<Eigen::Vector3d>& joint_positions, const std::vector<char>& joint_found, visualization_msgs::MarkerArray& marker_array)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = "kinect_depth_frame";
    marker.header.stamp = ros::Time::now();
    marker.ns = "body_" + std::to_string(body_id);
    marker.id = 0;
    marker.color.r = 0.;
    marker.color.g = 1.;
    marker.color.b = 0.;
    marker.color.a = 1.;
    marker.pose.orientation.w = 1.;
    marker.pose.orientation.x = 0.;
    marker.pose.orientation.y = 0.;
    marker.pose.orientation.z = 0.;
    marker.scale.x = radius * 2.;
    marker.scale.y = radius * 2.;
    marker.scale.z = radius * 2.;
    marker.lifetime = ros::Duration(0.1);

    marker.type = visualization_msgs::Marker::SPHERE;
    for (int i=0; i<joint_positions.size(); i++)
    {
        if (joint_found[i])
        {
            marker.action = visualization_msgs::Marker::ADD;
            tf::pointEigenToMsg(joint_positions[i], marker.pose.position);
        }
        else
        {
            marker.action = visualization_msgs::Marker::DELETE;
        }

        marker_array.markers.push_back(marker);
        marker.id++;
    }

    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.pose.position.x = 0.;
    marker.pose.position.y = 0.;
    marker.pose.position.z = 0.;
    marker.scale.x = radius;
    marker.points.resize(2);
    for (int i=0; i<edges.size(); i++)
    {
        const int i0 = edges[i].first;
        const int i1 = edges[i].second;

        if (joint_found[i0] && joint_found[i1])
        {
            marker.action = visualization_msgs::Marker::ADD;
            tf::pointEigenToMsg(joint_positions[i0], marker.points[0]);
            tf::pointEigenToMsg(joint_positions[i1], marker.points[1]);
        }
        else
        {
            marker.action = visualization_msgs::Marker::DELETE;
        }

        marker_array.markers.push_back(marker);
        marker.id++;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visualize_skeleton2");

    tf::TransformListener transform_listener;

    ros::Rate rate(30);

    const int num_bodies = 1;
    const int num_joints = kinect_network::Skeleton::getNumJoints();
    const std::vector<std::string>& joint_names = kinect_network::Skeleton::getJointNames();

    std::vector<Eigen::Vector3d> joint_positions(num_joints);
    std::vector<char> joint_found(num_joints);

    ros::NodeHandle node_handle("~");
    ros::Publisher publisher = node_handle.advertise<visualization_msgs::MarkerArray>("skeleton", 1);

    while (ros::ok())
    {
        visualization_msgs::MarkerArray marker_array;

        for (int i=0; i<num_bodies; i++)
        {
            for (int j=0; j<num_joints; j++)
            {
                // omit body id
                const std::string joint_name = joint_names[j];// + "_" + std::to_string(i);

                if (transform_listener.frameExists(joint_name))
                {
                    joint_found[j] = true;

                    std::string error_string;
                    ros::Time time;
                    transform_listener.getLatestCommonTime("kinect_depth_frame", joint_name, time, &error_string);

                    tf::StampedTransform transform;
                    try
                    {
                        transform_listener.lookupTransform("kinect_depth_frame", joint_name, time, transform);

                        tf::Vector3 position = transform.getOrigin();
                        joint_positions[j] = Eigen::Vector3d(position.x(), position.y(), position.z());
                    }
                    catch (const tf::TransformException& ex)
                    {
                        ROS_ERROR("%s", ex.what());
                    }
                }
                else
                {
                    joint_found[j] = false;
                }
            }

            visualizeSkeleton(i, joint_positions, joint_found, marker_array);
        }

        publisher.publish(marker_array);

        rate.sleep();
    }

    return 0;
}
