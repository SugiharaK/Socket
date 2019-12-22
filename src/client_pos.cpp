#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include <geometry_msgs/Pose.h>

int joints_num = 9;
double target_joints[9];
int pos_dimention = 3;
sensor_msgs::JointState target;
geometry_msgs::Pose grasp_position;

void Callback(const sensor_msgs::JointState::ConstPtr &msg)
{
  target = *msg;
  for (int i = 0; i < joints_num; i++)
  {
    target_joints[i] = target.position[i];
  }
  //for (int i = 0; i < joints_num; i++)
  //std::cout << target_joints[i] << std::endl;
}

void Grasp_Callback(const geometry_msgs::Pose::ConstPtr &msg)
{
  grasp_position = *msg;

  target_joints[joints_num] = grasp_position.position.x;
  target_joints[joints_num + 1] = grasp_position.position.y;
  target_joints[joints_num + 2] = grasp_position.position.z;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_client");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Subscriber sub = n.subscribe("/joint_states", 1000, Callback);
  ros::Subscriber sub_p = n.subscribe("/vs087/grasp_position", 1000, Grasp_Callback);
  ros::Publisher pub = n.advertise<std_msgs::String>("/vs087/move_permission", 1000);
  int sockfd;
  struct sockaddr_in addr;

  int port;
  ros::param::param<int>("~port", port, 1234);

  // ソケット生成
  if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
  {
    perror("socket");
  }

  // 送信先アドレス・ポート番号設定
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  addr.sin_addr.s_addr = inet_addr("192.168.0.89");
  //addr.sin_addr.s_addr = inet_addr("127.0.0.1");

  // サーバ接続
  connect(sockfd, (struct sockaddr *)&addr, sizeof(struct sockaddr_in));

  // データ送信
  double send_str[10000];
  char receive_str[1000];
  std_msgs::String move_permission;
  while (ros::ok())
  {
    ros::spinOnce();
    for (int i = 0; i < (joints_num + pos_dimention); i++)
    {
      printf("send:%f\n", target_joints[i]);
    }

    if (send(sockfd, target_joints, 10000, 0) < 0)
    {
      perror("send");
    }
    else
    {
      recv(sockfd, receive_str, 1000, 0);
      printf("receive:%s\n", receive_str);
      pub.publish(move_permission);
    }
  }

  // ソケットクローズ
  close(sockfd);

  return 0;
}