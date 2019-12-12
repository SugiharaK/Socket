#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"

int joints_num = 9;
double target_joints[9];
sensor_msgs::JointState target;

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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_client");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Subscriber sub = n.subscribe("/joint_states", 1000, Callback);

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
  double receive_str[10000];
  while (ros::ok())
  {
    ros::spinOnce();
    for (int i = 0; i < joints_num; i++)
      printf("send:%f\n", target_joints[i]);
    if (send(sockfd, target_joints, 10000, 0) < 0)
    {
      perror("send");
    }
    /*else
    {
      recv(sockfd, receive_str, 1000, 0);
      for (int i = 0; i < joints_num; i++)
        printf("receive:%f\n", receive_str[i]);
    }*/
  }

  // ソケットクローズ
  close(sockfd);

  return 0;
}