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

sensor_msgs::PointCloud2 pc;
int point_cloud[50000];
int points_num;
int width;
uint8_t *send_cloud;
int msg_len[2];

void Callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pc = *msg;
  point_cloud[0] = pc.row_step; //points_num
  point_cloud[1] = pc.width;    //width

  for (int i = 0; i < points_num; i++)
  {
    point_cloud[i + 2] = pc.data[i];
  }
  printf("callback:%d\n", point_cloud[0]);
  //for (int i = 0; i < joints_num; i++)
  //std::cout << target_joints[i] << std::endl;
}

int main(int argc, char **argv)
{
  int joint_num = 6;
  int finger_joint_num = 3;

  ros::init(argc, argv, "joint_and_pointcloud_server");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::string pc_src;
  ros::param::param<std::string>("~pc_src", pc_src, "/filtered_cloud");
  ros::Subscriber sub = n.subscribe(pc_src, 1000, Callback);

  //ros::Publisher pub = n.advertise<trajectory_msgs::JointTrajectory>("/twin/armR_controller/command", 1000);
  ros::Publisher pub = n.advertise<trajectory_msgs::JointTrajectory>("/vs087/arm_controller/command", 1000);
  ros::Publisher fpub = n.advertise<trajectory_msgs::JointTrajectory>("/vs087/gripper_controller/command", 1000);
  ros::Publisher pub2 = n.advertise<trajectory_msgs::JointTrajectory>("/robot2/arm_controller/command", 1000);
  ros::Publisher fpub2 = n.advertise<trajectory_msgs::JointTrajectory>("/robot2/gripper_controller/command", 1000);
  int sockfd;
  int client_sockfd;
  int client_sockfd2;
  struct sockaddr_in addr;

  socklen_t len = sizeof(struct sockaddr_in);
  struct sockaddr_in from_addr;

  double buf[9];
  double buf2[9];
  int joint_msg_len = 9 * 8;

  // 受信バッファ初期化
  memset(buf, 0, sizeof(buf));
  memset(buf2, 0, sizeof(buf2));

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
  std::cout << "before accept" << std::endl;
  // クライアントからのコネクト要求待ち
  if ((client_sockfd = accept(sockfd, (struct sockaddr *)&from_addr, &len)) < 0)
  {
    perror("accept");
  }
  std::cout << "accept1" << client_sockfd << std::endl;
  if ((client_sockfd2 = accept(sockfd, (struct sockaddr *)&from_addr, &len)) < 0)
  {
    perror("accept");
  }
  std::cout << "accept2" << client_sockfd2 << std::endl;

  // 受信
  int rsize;
  int rsize2;
  trajectory_msgs::JointTrajectory target;
  trajectory_msgs::JointTrajectory finger_target;
  trajectory_msgs::JointTrajectory target2;
  trajectory_msgs::JointTrajectory finger_target2;

  geometry_msgs::Pose vs087_position;
  geometry_msgs::Pose robot2_position;

  target.header.frame_id = "world";
  finger_target.header.frame_id = "world";
  /*std::vector<std::string> joints = {"joint1",
                                     "joint2",
                                     "joint3",
                                     "joint4",
                                     "joint5",
                                     "joint6"};*/
  std::vector<std::string> joints = {
      "joint_1",
      "joint_2",
      "joint_3",
      "joint_4",
      "joint_5",
      "joint_6",
  };
  std::vector<std::string> finger_joints = {"finger_R_joint", "finger_L_joint", "finger_3rd_joint"};

  for (int i = 0; i < joint_num; i++)
  {
    target.joint_names.push_back(joints[i]);
  }
  for (int i = 0; i < finger_joint_num; i++)
  {
    finger_target.joint_names.push_back(finger_joints[i]);
  }

  trajectory_msgs::JointTrajectoryPoint init_point;
  target.points.push_back(init_point);
  finger_target.points.push_back(init_point);
  target.points[0].time_from_start = ros::Duration(0.01);
  finger_target.points[0].time_from_start = ros::Duration(0.01);

  target2 = target;
  finger_target2 = finger_target;

  //initialaize
  //rsize = recv(client_sockfd, buf, 10000, 0);
  for (int i = 0; i < joint_num; i++)
  {
    target.points[0].positions.push_back(0.0);
    //printf("receive:%f\n", buf[i]);
  }
  for (int i = joint_num; i < (joint_num + finger_joint_num); i++)
  {
    finger_target.points[0].positions.push_back(0.0);
    //printf("receive:%f\n", buf[i]);
  }

  /*for (int i = 0; i < (joint_num + finger_joint_num); i++)
  {
    printf("send:%f\n", buf[i]);
  }
  write(client_sockfd, buf, rsize);*/

  //rsize = recv(client_sockfd2, buf2, 10000, 0);

  for (int i = 0; i < joint_num; i++)
  {
    target2.points[0].positions.push_back(0.0);

    //printf("receive2:%f\n", buf[i]);
  }
  for (int i = joint_num; i < (joint_num + finger_joint_num); i++)
  {
    finger_target2.points[0].positions.push_back(0.0);
    //printf("receive2:%f\n", buf2[i]);
  }

  /*for (int i = 0; i < (joint_num + finger_joint_num); i++)
  {
    printf("send2:%f\n", buf2[i]);
  }
  write(client_sockfd2, buf2, rsize);*/
  std::cout << "initial end" << std::endl;
  ros::Rate rate(2);
  int buf_cloud[50000];

  // プロセスを作成する
  pid_t pid;
  pid = fork();
  // プロセスの作成に失敗
  if (pid < 0)
  {
    perror("fork()");
    return -1;
  }

  //loop
  while (ros::ok())
  {
    ros::spinOnce();
    if (pid == 0)
    {
      //robot1
      rsize = recv(client_sockfd, buf, joint_msg_len, 0);

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
        for (int i = 0; i < joint_num; i++)
        {
          target.points[0].positions[i] = buf[i];
          printf("receive:%f\n", buf[i]);
        }
        for (int i = joint_num; i < (joint_num + finger_joint_num); i++)
        {
          finger_target.points[0].positions[i - joint_num] = buf[i];
          printf("receive:%f\n", buf[i]);
        }

        target.header.stamp = ros::Time::now();
        finger_target.header.stamp = ros::Time::now();
        pub.publish(target);
        fpub.publish(finger_target);
        //ros::spinOnce();
        //sleep(1);
      }
      //sleep(0.2);
      //point_cloud1

      printf("pc_num:%d\n", points_num);
      for (int i = 0; i < 5; i++)
      {
        printf("pc_send:%d", point_cloud[i]);
      }
      printf("\n");

      for (int i = 0; i < point_cloud[0] + 2; i++)
      {
        buf_cloud[i] = point_cloud[i];
      }
      msg_len[0] = points_num = buf_cloud[0];
      msg_len[1] = width = buf_cloud[1];
      send_cloud = new uint8_t[points_num];
      for (int i = 0; i < points_num; i++)
      {
        send_cloud[i] = buf_cloud[i + 2];
      }

      if (send(client_sockfd, msg_len, sizeof(msg_len), 0) < 0)
      {
        perror("send");
      }
      printf("msg_len:%d\n", msg_len[0]);
      if (send(client_sockfd, send_cloud, msg_len[0], 0) < 0)
      {
        perror("send");
      }
      //sleep(0.001 * points_num);
      delete[] send_cloud;
    }
    else
    {
      //robot2
      rsize2 = recv(client_sockfd2, buf2, joint_msg_len, 0);

      if (rsize2 == 0)
      {
        break;
      }
      else if (rsize2 == -1)
      {
        perror("recv");
      }
      else
      {
        for (int i = 0; i < joint_num; i++)
        {
          target2.points[0].positions[i] = buf2[i];
          printf("receive2:%f\n", buf2[i]);
        }
        for (int i = joint_num; i < (joint_num + finger_joint_num); i++)
        {
          finger_target2.points[0].positions[i - joint_num] = buf2[i];
          printf("receive2:%f\n", buf2[i]);
        }
        target2.header.stamp = ros::Time::now();
        finger_target2.header.stamp = ros::Time::now();
        pub2.publish(target2);
        fpub2.publish(finger_target2);
        //ros::spinOnce();
      }
      //sleep(0.2);
      //point_cloud2
      printf("pc_num2:%d\n", points_num);
      for (int i = 0; i < 5; i++)
      {
        printf("pc_send2:%d", point_cloud[i]);
      }
      printf("\n");

      for (int i = 0; i < point_cloud[0] + 2; i++)
      {
        buf_cloud[i] = point_cloud[i];
      }
      msg_len[0] = points_num = buf_cloud[0];
      msg_len[1] = width = buf_cloud[1];
      send_cloud = new uint8_t[points_num];
      for (int i = 0; i < points_num; i++)
      {
        send_cloud[i] = buf_cloud[i + 2];
      }
      if (send(client_sockfd2, msg_len, sizeof(msg_len), 0) < 0)
      {
        perror("send");
      }
      printf("msg_len2:%d\n", msg_len[0]);
      if (send(client_sockfd2, send_cloud, msg_len[0], 0) < 0)
      {
        perror("send");
      }
      delete[] send_cloud;
      //rate.sleep();
    }
  }
  // ソケットクローズ
  close(client_sockfd);
  close(client_sockfd2);
  close(sockfd);

  return 0;
}