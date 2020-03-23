#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include <geometry_msgs/Pose.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointField.h"

#include <thread>

namespace server_twin_point_cloud_thred
{
class ServerTwinPointCloudThred
{
public:
  void PointCloudCallback();
  ServerTwinPointCloudThred::Server();

private:
  ServerTwinPointCloudThred::ServerTwinPointCloudThred();
  ServerTwinPointCloudThred::robot1();
  ServerTwinPointCloudThred::robot2();

private:
  sensor_msgs::PointCloud2;
  int point_cloud[50000];
  int points_num;
  int width;
  uint8_t *send_cloud;
  uint8_t *send_cloud2;
  int msg_len[2];
  int msg_len2[2];
  ros::Subscriber sub;
  ros::Publisher pub;
  ros::Publisher fpub;
  ros::Publisher pub2;
  ros::Publisher fpub2;
  int joint_num;
  int finger_joint_num;
  std::string pc_src;
  int sockfd;
  int client_sockfd;
  int client_sockfd2;
  struct sockaddr_in addr;

  socklen_t len;
  struct sockaddr_in from_addr;

  double buf[9];
  double buf2[9];
  int joint_msg_len;

  int rsize;
  int rsize2;
  trajectory_msgs::JointTrajectory target;
  trajectory_msgs::JointTrajectory finger_target;
  trajectory_msgs::JointTrajectory target2;
  trajectory_msgs::JointTrajectory finger_target2;

  geometry_msgs::Pose vs087_position;
  geometry_msgs::Pose robot2_position;

  trajectory_msgs::JointTrajectoryPoint init_point;

  int buf_cloud[50000];
  int buf_cloud2[50000];
}
} // namespace server_twin_point_cloud_thred