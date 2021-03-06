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
#include "sensor_msgs/PointField.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointcloud_client");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Publisher pubpc = n.advertise<sensor_msgs::PointCloud2>("/socket_pc", 10000);
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

  // 受信
  int rsize;
  sensor_msgs::PointCloud2 point_cloud;
  sensor_msgs::PointCloud2 init_cloud;
  sensor_msgs::PointField init_field;

  //initialaize
  std::cout << __LINE__ << std::endl;
  int buf[100000];
  int msg_len;

  // 受信バッファ初期化
  memset(buf, 0, sizeof(buf));

  /**/ point_cloud.fields.push_back(init_field);
  point_cloud.fields.push_back(init_field);
  point_cloud.fields.push_back(init_field);
  std::cout << __LINE__ << std::endl;
  init_field.offset = 0;
  init_field.name = "x";
  init_field.datatype = 7;
  init_field.count = 1;
  std::cout << __LINE__ << std::endl;
  point_cloud.fields[0] = init_field;
  std::cout << __LINE__ << std::endl;
  init_field.offset = 4;
  init_field.name = "y";
  point_cloud.fields[1] = init_field;
  std::cout << __LINE__ << std::endl;
  init_field.offset = 8;
  init_field.name = "z";
  point_cloud.fields[2] = init_field;
  std::cout << __LINE__ << std::endl;

  point_cloud.header.frame_id = "world";
  point_cloud.point_step = 16;
  point_cloud.height = 1;
  point_cloud.is_dense = 1;

  std::cout << __LINE__ << std::endl;
  while (ros::ok())
  {

    recv(sockfd, &msg_len, 4, 0);
    printf("msg:%d\n", msg_len);
    rsize = recv(sockfd, buf, msg_len, 0);
    sleep(0.2);

    if (rsize == 0)
    {
      break;
    }
    else if (rsize == -1)
    {
      perror("recv");
    }
    else
    {
      point_cloud.data = init_cloud.data;
      int bufsize = buf[0];
      int width = buf[1];
      printf("size:%d\n", bufsize);
      for (int i = 2; i < bufsize + 2; i++)
      {
        point_cloud.data.push_back(buf[i]);
        //printf("receive:%d\n", buf[i]);
      }
      printf("receive:%d\n", buf[1]);
      point_cloud.header.stamp = ros::Time::now();
      point_cloud.row_step = bufsize;
      point_cloud.width = width;
      pubpc.publish(point_cloud);
      ros::spinOnce();

      // 応答
      /* for (int i = 0; i < 5; i++)
      {
        printf("send:%d", buf[i]);
      }
      printf("\n");
      write(client_sockfd, buf, 1000000000);*/
    }
  }

  // ソケットクローズ
  close(sockfd);

  return 0;
}