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
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointField.h"

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
  int joint_num = 6;
  int finger_joint_num = 3;

  ros::init(argc, argv, "pointcloud_server");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Subscriber sub = n.subscribe("/downsampled_cloud", 1000, Callback);

  std::string robot_name;
  int port;
  ros::param::param<std::string>("~robot_name", robot_name, "vs087");
  ros::param::param<int>("~port", port, 1234);

  int sockfd;
  int client_sockfd;
  struct sockaddr_in addr;

  socklen_t len = sizeof(struct sockaddr_in);
  struct sockaddr_in from_addr;
  std::cout << __LINE__ << std::endl;
  int buf[1000000];
  std::cout << __LINE__ << std::endl;
  // 受信バッファ初期化
  memset(buf, 0, sizeof(buf));

  // ソケット生成
  if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
  {
    perror("socket");
  }

  // 待ち受け用IP・ポート番号設定
  addr.sin_family = AF_INET;
  addr.sin_port = htons(1234);
  addr.sin_addr.s_addr = INADDR_ANY;
  // バインド
  if (bind(sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
  {
    perror("bind");
  }

  // 受信待ち
  if (listen(sockfd, SOMAXCONN) < 0)
  {
    perror("listen");
  }

  // クライアントからのコネクト要求待ち
  if ((client_sockfd = accept(sockfd, (struct sockaddr *)&from_addr, &len)) < 0)
  {
    perror("accept");
  }
  std::cout << __LINE__ << std::endl;
  // データ送信
  while (ros::ok())
  {
    ros::spinOnce();
    printf("pc_num:%d\n", points_num);
    for (int i = 0; i < 5; i++)
    {
      printf("send:%d", point_cloud[i]);
    }
    printf("\n");
    int msg_len = points_num * 2;
    if (send(client_sockfd, point_cloud, 30000, 0) < 0)
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
  close(client_sockfd);
  close(sockfd);

  return 0;
}