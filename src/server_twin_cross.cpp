#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <math.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include <geometry_msgs/Pose.h>

#define epsion 2.22045e-016
#define gripper_length 0.18

bool judgeIentersected(double ax, double ay, double bx, double by, double cx, double cy, double dx, double dy)
{
  double ta = (cx - dx) * (ay - cy) + (cy - dy) * (cx - ax);
  double tb = (cx - dx) * (by - cy) + (cy - dy) * (cx - bx);
  double tc = (ax - bx) * (cy - ay) + (ay - by) * (ax - cx);
  double td = (ax - bx) * (dy - ay) + (ay - by) * (ax - dx);

  return tc * td < 0 && ta * tb < 0;
  // return tc * td <= 0 && ta * tb <= 0; // 端点を含む場合
}
double getDistance(double x, double y, double x2, double y2)
{
  double distance = sqrt((x2 - x) * (x2 - x) + (y2 - y) * (y2 - y));

  return distance;
}

int main(int argc, char **argv)
{
  int joint_num = 6;
  int finger_joint_num = 3;
  int pos_dimention = 3;
  int move_frag = 0;

  ros::init(argc, argv, "joint_server");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::string robot_name;
  int port;
  ros::param::param<std::string>("~robot_name", robot_name, "vs087");
  ros::param::param<int>("~port", port, 1234);

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

  double buf[10000];
  double buf2[10000];
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
  std::cout << "accept1" << std::endl;
  if ((client_sockfd2 = accept(sockfd, (struct sockaddr *)&from_addr, &len)) < 0)
  {
    perror("accept");
  }
  std::cout << "accept2" << std::endl;

  // 受信
  int rsize;
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
  rsize = recv(client_sockfd, buf, 10000, 0);
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

  for (int i = 0; i < (joint_num + finger_joint_num); i++)
  {
    printf("send:%f\n", buf[i]);
  }

  write(client_sockfd, buf, rsize);

  rsize = recv(client_sockfd2, buf2, 10000, 0);

  for (int i = 0; i < joint_num; i++)
  {
    target2.points[0].positions.push_back(buf2[i]);

    printf("receive2:%f\n", buf[i]);
  }
  for (int i = joint_num; i < (joint_num + finger_joint_num); i++)
  {
    finger_target2.points[0].positions.push_back(buf2[i]);
    printf("receive2:%f\n", buf2[i]);
  }

  for (int i = 0; i < (joint_num + finger_joint_num); i++)
  {
    printf("send2:%f\n", buf2[i]);
  }
  write(client_sockfd2, buf2, rsize);

  //loop
  while (ros::ok())
  {
    rsize = recv(client_sockfd, buf, 10000, 0);

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
      vs087_position.position.x = buf[joint_num + finger_joint_num];
      vs087_position.position.y = buf[joint_num + finger_joint_num + 1];
      vs087_position.position.z = buf[joint_num + finger_joint_num + 2];
      for (int i = (joint_num + finger_joint_num); i < (joint_num + finger_joint_num + pos_dimention); i++)
      {
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

    //robot2
    rsize = recv(client_sockfd2, buf2, 10000, 0);

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
        target2.points[0].positions[i] = buf2[i];
        printf("receive2:%f\n", buf2[i]);
      }
      for (int i = joint_num; i < (joint_num + finger_joint_num); i++)
      {
        finger_target2.points[0].positions[i - joint_num] = buf2[i];
        printf("receive2:%f\n", buf2[i]);
      }
      robot2_position.position.x = buf2[joint_num + finger_joint_num];
      robot2_position.position.y = buf2[joint_num + finger_joint_num + 1];
      robot2_position.position.z = buf2[joint_num + finger_joint_num + 2];
      for (int i = (joint_num + finger_joint_num); i < (joint_num + finger_joint_num + pos_dimention); i++)
      {
        printf("receive2:%f\n", buf2[i]);
      }
      target2.header.stamp = ros::Time::now();
      finger_target2.header.stamp = ros::Time::now();
      pub2.publish(target2);
      fpub2.publish(finger_target2);
      ros::spinOnce();

      // 応答
      /*for (int i = 0; i < (joint_num + finger_joint_num); i++)
      {
        printf("send:%f\n", buf[i]);
      }
      write(client_sockfd, buf, rsize);*/
    }

    //cross judge
    int crs_judge = judgeIentersected(0.0, 0.0, vs087_position.position.x, vs087_position.position.y, 0.0, 1.0, robot2_position.position.x, robot2_position.position.y + 1.0);
    double dist = getDistance(vs087_position.position.x, vs087_position.position.y, robot2_position.position.x, robot2_position.position.y + 1.0);
    //if (!(fabs(vs087_position.position.x) < epsion && fabs(vs087_position.position.y) < epsion) || !(fabs(robot2_position.position.x) < epsion && fabs(robot2_position.position.y) < epsion))
    //{
    printf("judge:%d,%f\n", !crs_judge, dist);
    if (!crs_judge && (dist > gripper_length))
    {
      write(client_sockfd, "true", 1000);
      write(client_sockfd2, "true", 1000);
      printf("ok\n");
    }
    /*else if (move_frag % 2 == 0)
    {
      write(client_sockfd, "true", 1000);
      write(client_sockfd2, "false", 1000);
      printf("cross:vs087\n");
      move_frag++;
    }*/
    else
    {
      write(client_sockfd, "false", 1000);
      write(client_sockfd2, "false", 1000);
      printf("cross:robot2\n");
      move_frag++;
    }
    //}
  }

  // ソケットクローズ
  close(client_sockfd);
  close(sockfd);

  return 0;
}