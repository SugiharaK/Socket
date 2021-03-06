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

int joint_num = 6;
int finger_num = 3;
int joint_sum = 9;
double target_joints[9];
int joint_msg_len = 9 * 8;
int msg_len[2];
sensor_msgs::JointState target;

void Callback(const sensor_msgs::JointState::ConstPtr &msg)
{
  target = *msg;
  for (int i = 0; i < joint_num; i++)
  {
    target_joints[i] = target.position[i + finger_num];
  }
  for (int i = 0; i < finger_num; i++)
  {
    target_joints[i + joint_num] = target.position[i];
  }
  //for (int i = 0; i < joints_num; i++)
  //std::cout << target_joints[i] << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_and_pointcloud_client");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Publisher pubpc = n.advertise<sensor_msgs::PointCloud2>("/socket_pc", 10000);
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
  addr.sin_addr.s_addr = inet_addr("192.168.0.2");
  //addr.sin_addr.s_addr = inet_addr("192.168.0.247");
  //addr.sin_addr.s_addr = inet_addr("192.168.0.89");
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
  uint8_t buf[50000];
  uint8_t marge_buf[50000];
  std::cout << __LINE__ << std::endl;
  // 受信バッファ初期化
  memset(buf, 0, sizeof(buf));
  memset(marge_buf, 0, sizeof(marge_buf));

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
  ros::Rate rate(2);

  int recvd_buf;
  int bufsize;
  int width;

  while (ros::ok())
  {
    //joint
    ros::spinOnce();
    for (int i = 0; i < joint_sum; i++)
      printf("send:%f\n", target_joints[i]);
    if (send(sockfd, target_joints, joint_msg_len, 0) < 0)
    {
      perror("send");
    }
    //point_cloud
    //sleep(1);
    recv(sockfd, msg_len, sizeof(msg_len), 0);
    printf("msg_len:%d\n", msg_len[0]);
    if (msg_len[0] > 50000)
    {
      recv(sockfd, marge_buf, sizeof(marge_buf), 0);
    }
    else
    {
      recvd_buf = 0;
      while (recvd_buf < msg_len[0])
      {
        rsize = recv(sockfd, buf, msg_len[0] - recvd_buf, 0);
        memmove(marge_buf + recvd_buf, buf, rsize);
        recvd_buf += rsize;
        std::cout << "receved_buf_length:" << recvd_buf << std::endl;
        if (rsize == 0)
        {
          break;
        }
        else if (rsize == -1)
        {
          perror("recv");
        }
      }

      //sleep(0.2);
      if (rsize == 0)
      {
        break;
      }
      else
      {
        //point_cloud.data = init_cloud.data;
        point_cloud.data.clear();
        bufsize = msg_len[0];
        width = msg_len[1];
        printf("size:%d\n", bufsize);
        for (int i = 0; i < bufsize; i++)
        {
          //printf("%d ",i);
          point_cloud.data.push_back(marge_buf[i]);
          //sleep(0.0008);
          //printf("receive:%d\n", buf[i]);
        }
        printf("receive:%d\n", marge_buf[0]);
        point_cloud.header.stamp = ros::Time::now();
        point_cloud.row_step = bufsize;
        point_cloud.width = width;
        pubpc.publish(point_cloud);

        // 応答
        /* for (int i = 0; i < 5; i++)
      {
        printf("send:%d", buf[i]);
      }
      printf("\n");
      write(client_sockfd, buf, 1000000000);*/
      }
    }
    ros::spinOnce();
    //rate.sleep();
  }

  // ソケットクローズ
  close(sockfd);

  return 0;
}