#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/PointCloud2.h"

int joints_num = 9;
double target_joints[9];
sensor_msgs::JointState target;
sensor_msgs::PointCloud2 pc;
int point_cloud[100000];
int points_num;
int width;

void Callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pc = *msg;
  points_num = pc.row_step;
  width = pc.width;
  point_cloud[0] = points_num;
  point_cloud[1] = width;
  for (int i = 2; i < points_num + 2; i++)
  {
    point_cloud[i] = pc.data[i - 2];
  }
  printf("callback:%d\n", point_cloud[1]);
  //for (int i = 0; i < joints_num; i++)
  //std::cout << target_joints[i] << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_client");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Subscriber sub = n.subscribe("/downsampled_cloud", 1000, Callback);

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
  int receive_str[100000];
  while (ros::ok())
  {
    ros::spinOnce();
    printf("pc_num:%d\n", points_num);
    for (int i = 0; i < 5; i++)
    {
      printf("send:%d", point_cloud[i]);
    }
    printf("\n");

    if (send(sockfd, point_cloud, 100000, 0) < 0)
    {
      perror("send");
    }
    /* else
    {
      recv(sockfd, receive_str, 1000000000, 0);
      for (int i = 0; i < 5; i++)
        printf("receive:%d", receive_str[i]);
    }
    printf("\n");*/
  }

  // ソケットクローズ
  close(sockfd);

  return 0;
}