#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>

class PoseListener
{
public:
    PoseListener()
    {
        // Subscribe to the /robot_pose_ekf/odom_combined topic
        sub_ = nh_.subscribe("/robot_pose_ekf/odom_combined", 10, &PoseListener::poseCallback, this);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    tf::TransformBroadcaster br_;

    // Callback function to process the pose data
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
        // Extract position
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double z = msg->pose.pose.position.z;

        // Extract orientation (quaternion)
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;

        // Print the pose information
        ROS_INFO("Position: [x: %f, y: %f, z: %f]", x, y, z);
        ROS_INFO("Orientation: [qx: %f, qy: %f, qz: %f, qw: %f]", qx, qy, qz, qw);

        // Create a Transform object and set its values
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(x, y, z));
        tf::Quaternion quaternion(qx, qy, qz, qw);
        transform.setRotation(quaternion);

        // Broadcast the transformation
        br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom_combined", "base_footprint"));
    }
};

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "odom_to_base");

    // Create the PoseListener object
    PoseListener listener;

    // Spin to keep the node running
    ros::spin();

    return 0;
}
