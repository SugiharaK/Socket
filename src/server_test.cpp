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

int main(int argc, char **argv)
{
  int joint_num = 6;
  int finger_joint_num = 3;

  ros::init(argc, argv, "joint_server");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::string robot_name;
  int port;
  ros::param::param<std::string>("~robot_name", robot_name, "vs087");
  ros::param::param<int>("~port", port, 1234);

  //ros::Publisher pub = n.advertise<trajectory_msgs::JointTrajectory>("/twin/armR_controller/command", 1000);
  ros::Publisher pub = n.advertise<trajectory_msgs::JointTrajectory>("/" + robot_name + "/arm_controller/command", 1000);
  ros::Publisher fpub = n.advertise<trajectory_msgs::JointTrajectory>("/" + robot_name + "/gripper_controller/command", 1000);
  int sockfd;
  int client_sockfd;
  struct sockaddr_in addr;

  socklen_t len = sizeof(struct sockaddr_in);
  struct sockaddr_in from_addr;

  double buf[8000];
  int msg_len = 9 * 8;
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

  // 受信
  int rsize;
  trajectory_msgs::JointTrajectory target;
  trajectory_msgs::JointTrajectory finger_target;
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

  //initialaize
  rsize = recv(client_sockfd, buf, msg_len, 0);
  for (int i = 0; i < joint_num; i++)
  {
    target.points[0].positions.push_back(buf[i]);
    printf("receive:%f\n", buf[i]);
  }
  for (int i = joint_num; i < (joint_num + finger_joint_num); i++)
  {
    finger_target.points[0].positions.push_back(buf[i]);
    printf("receive:%f\n", buf[i]);
  }
  /*
  for (int i = 0; i < (joint_num + finger_joint_num); i++)
  {
    printf("send:%f\n", buf[i]);
  }
  write(client_sockfd, buf, rsize);
*/
  while (ros::ok())
  {
    rsize = recv(client_sockfd, buf, 8000, 0);

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
      ros::spinOnce();

      // 応答
      /*for (int i = 0; i < (joint_num + finger_joint_num); i++)
      {
        printf("send:%f\n", buf[i]);
      }
      write(client_sockfd, buf, rsize);*/
    }
  }

  // ソケットクローズ
  close(client_sockfd);
  close(sockfd);

  return 0;
}