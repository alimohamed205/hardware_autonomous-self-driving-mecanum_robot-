#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <cmath>

// Create odometry data publishers
ros::Publisher odom_data_pub;
ros::Publisher odom_data_pub_quat;
nav_msgs::Odometry odomNew;
nav_msgs::Odometry odomOld;

// Initial pose
const double initialX = 0.0;
const double initialY = 0.0;
const double initialTheta = 0.00000000001;
const double PI = 3.14159265358979323846;

// Robot physical constants
const double TICKS_PER_REVOLUTION = 620;
const double WHEEL_RADIUS = 0.033;
const double WHEEL_BASE = 0.17;
const double WHEEL_TRACK = 0.17;
const double TICKS_PER_METER = 3100;

// Distance all wheels have traveled
double distanceFrontLeft = 0;
double distanceFrontRight = 0;
double distanceBackLeft = 0;
double distanceBackRight = 0;

// Flag to see if initial pose has been received
bool initialPoseReceived = true;

using namespace std;

// Get initialpose message from either Rviz clicks or a manual pose publisher
void set_initial_2d(const geometry_msgs::PoseStamped &rvizClick) {
  odomOld.pose.pose.position.x = rvizClick.pose.position.x;
  odomOld.pose.pose.position.y = rvizClick.pose.position.y;
  odomOld.pose.pose.orientation.z = rvizClick.pose.orientation.z;
  initialPoseReceived = true;
}

// Calculate the distance the front left wheel has traveled since the last cycle
void calculate_front_left_distance(const std_msgs::Int32& frontLeftCount) {
  static int lastCountFL = 0;
  if (lastCountFL != 0) {
    int frontLeftTicks = frontLeftCount.data - lastCountFL;
    if (frontLeftTicks > 10000) {
      frontLeftTicks = 0 - (65535 - frontLeftTicks);
    } else if (frontLeftTicks < -10000) {
      frontLeftTicks = 65535 - frontLeftTicks;
    }
    distanceFrontLeft = frontLeftTicks / TICKS_PER_METER;
  }
  lastCountFL = frontLeftCount.data;
}

// Calculate the distance the front right wheel has traveled since the last cycle
void calculate_front_right_distance(const std_msgs::Int32& frontRightCount) {
  static int lastCountFR = 0;
  if (lastCountFR != 0) {
    int frontRightTicks = frontRightCount.data - lastCountFR;
    if (frontRightTicks > 10000) {
      frontRightTicks = 0 - (65535 - frontRightTicks);
    } else if (frontRightTicks < -10000) {
      frontRightTicks = 65535 - frontRightTicks;
    }
    distanceFrontRight = frontRightTicks / TICKS_PER_METER;
  }
  lastCountFR = frontRightCount.data;
}

// Calculate the distance the back left wheel has traveled since the last cycle
void calculate_back_left_distance(const std_msgs::Int32& backLeftCount) {
  static int lastCountBL = 0;
  if (lastCountBL != 0) {
    int backLeftTicks = backLeftCount.data - lastCountBL;
    if (backLeftTicks > 10000) {
      backLeftTicks = 0 - (65535 - backLeftTicks);
    } else if (backLeftTicks < -10000) {
      backLeftTicks = 65535 - backLeftTicks;
    }
    distanceBackLeft = backLeftTicks / TICKS_PER_METER;
  }
  lastCountBL = backLeftCount.data;
}

// Calculate the distance the back right wheel has traveled since the last cycle
void calculate_back_right_distance(const std_msgs::Int32& backRightCount) {
  static int lastCountBR = 0;
  if (lastCountBR != 0) {
    int backRightTicks = backRightCount.data - lastCountBR;
    if (backRightTicks > 10000) {
      backRightTicks = 0 - (65535 - backRightTicks);
    } else if (backRightTicks < -10000) {
      backRightTicks = 65535 - backRightTicks;
    }
    distanceBackRight = backRightTicks / TICKS_PER_METER;
  }
  lastCountBR = backRightCount.data;
}

// Publish a nav_msgs::Odometry message in quaternion format
void publish_quat() {
  tf2::Quaternion q;
  q.setRPY(0, 0, odomNew.pose.pose.orientation.z);
  q.normalize();
  nav_msgs::Odometry quatOdom;
  quatOdom.header.stamp = odomNew.header.stamp;
  quatOdom.header.frame_id = "odom_combined";
  quatOdom.child_frame_id = "base_link";
  quatOdom.pose.pose.position.x = odomNew.pose.pose.position.x;
  quatOdom.pose.pose.position.y = odomNew.pose.pose.position.y;
  quatOdom.pose.pose.position.z = odomNew.pose.pose.position.z;
  quatOdom.pose.pose.orientation.x = q.x();
  quatOdom.pose.pose.orientation.y = q.y();
  quatOdom.pose.pose.orientation.z = q.z();
  quatOdom.pose.pose.orientation.w = q.w();
  quatOdom.twist.twist.linear.x = odomNew.twist.twist.linear.x;
  quatOdom.twist.twist.linear.y = odomNew.twist.twist.linear.y;
  quatOdom.twist.twist.linear.z = odomNew.twist.twist.linear.z;
  quatOdom.twist.twist.angular.x = odomNew.twist.twist.angular.x;
  quatOdom.twist.twist.angular.y = odomNew.twist.twist.angular.y;
  quatOdom.twist.twist.angular.z = odomNew.twist.twist.angular.z;
  for (int i = 0; i < 36; i++) {
    if (i == 0 || i == 7 || i == 14) {
      quatOdom.pose.covariance[i] = 0.01;
    } else if (i == 21 || i == 28 || i == 35) {
      quatOdom.pose.covariance[i] += 0.1;
    } else {
      quatOdom.pose.covariance[i] = 0;
    }
  }
  odom_data_pub_quat.publish(quatOdom);
}

// Update odometry information
void update_odom(tf2_ros::Buffer& tfBuffer) {
  // Calculate velocities based on mecanum wheel kinematics
  double v_x = (distanceFrontLeft + distanceFrontRight + distanceBackLeft + distanceBackRight) / 4;
  double v_y = (-distanceFrontLeft + distanceFrontRight + distanceBackLeft - distanceBackRight) / 4;
  double omega = (-distanceFrontLeft + distanceFrontRight - distanceBackLeft + distanceBackRight) / (4 * (WHEEL_BASE + WHEEL_TRACK));

  double dt = (odomNew.header.stamp - odomOld.header.stamp).toSec();
  if (dt == 0) return; // Avoid division by zero

  // Update position and orientation
  odomNew.pose.pose.position.x = odomOld.pose.pose.position.x + v_x * dt;
  odomNew.pose.pose.position.y = odomOld.pose.pose.position.y + v_y * dt;
  odomNew.pose.pose.orientation.z = odomOld.pose.pose.orientation.z + omega * dt;

  // Normalize the quaternion
  tf2::Quaternion q;
  q.setRPY(0, 0, odomNew.pose.pose.orientation.z);
  q.normalize();
  odomNew.pose.pose.orientation.x = q.x();
  odomNew.pose.pose.orientation.y = q.y();
  odomNew.pose.pose.orientation.z = q.z();
  odomNew.pose.pose.orientation.w = q.w();

  // Set twist (linear and angular velocities)
  odomNew.twist.twist.linear.x = v_x / dt;
  odomNew.twist.twist.linear.y = v_y / dt;
  odomNew.twist.twist.angular.z = omega / dt;

  // Update timestamp
  odomNew.header.stamp = ros::Time::now();

  // Publish odometry messages
  odom_data_pub.publish(odomNew);
  publish_quat();

  // Save current odom as old odom for next cycle
  odomOld = odomNew;
}

// Main function
int main(int argc, char** argv) {
  ros::init(argc, argv, "odom_publisher_node");
  ros::NodeHandle node;

  // Subscribe to wheel tick counts
  ros::Subscriber subForTicksFL = node.subscribe("left_front_wheel_ticks", 100, calculate_front_left_distance);
  ros::Subscriber subForTicksFR = node.subscribe("right_front_wheel_ticks", 100, calculate_front_right_distance);
  ros::Subscriber subForTicksBL = node.subscribe("left_back_wheel_ticks", 100, calculate_back_left_distance);
  ros::Subscriber subForTicksBR = node.subscribe("right_back_wheel_ticks", 100, calculate_back_right_distance);

  // Subscribe to initial pose
  ros::Subscriber initialPose = node.subscribe("initial_2d", 100, set_initial_2d);

  // Advertise odometry topics
  odom_data_pub = node.advertise<nav_msgs::Odometry>("odom_data", 100);
  odom_data_pub_quat = node.advertise<nav_msgs::Odometry>("odom_data_quat", 100);

  // Setup transform listener
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  // Initialize odometry message headers
  odomOld.header.frame_id = "odom_combined";
  odomOld.child_frame_id = "base_link";
  odomNew.header.frame_id = "odom_combined";
  odomNew.child_frame_id = "base_link";

  // Set initial pose
  odomOld.pose.pose.position.x = initialX;
  odomOld.pose.pose.position.y = initialY;
  odomOld.pose.pose.orientation.z = initialTheta;

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    ros::spinOnce();

    if (initialPoseReceived) {
      try {
        // Wait for transform to become available
        tfBuffer.canTransform("odom_combined", "base_link", ros::Time(0), ros::Duration(1.0));
        update_odom(tfBuffer);
      } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
      }
    }
    loop_rate.sleep();
  }
  return 0;
}
