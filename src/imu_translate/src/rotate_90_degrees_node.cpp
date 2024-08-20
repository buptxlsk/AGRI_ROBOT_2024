#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <cmath>
#include "pid.h"

#define LEFT 1
#define RIGHT -1 

ros::Publisher vel_pub, success_pub;
double rotate_speed=0.005;
geometry_msgs::Twist target_vel;
std_msgs::Bool success_msg;  // 表示旋转成功的消息
int rotate_command = 0;  // 当前的旋转命令
double target_yaw, current_yaw, initial_yaw;  // 目标航向角、当前航向角、初始航向角
PID pid;  // PID控制器实例

// RPY数据回调函数
void rpyCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
  // current_yaw = msg->z;  // 直接使用从imu_rpy发布的yaw角度

  // if (rotate_command != 0)
  // {
  //   double angle_diff = fabs(current_yaw - target_yaw);
  //   if (angle_diff > M_PI)  // 处理角度的环绕问题
  //     angle_diff = 2 * M_PI - angle_diff;



  //   ROS_INFO("Rotating %s: Current Yaw: %.2f, Target Yaw: %.2f", rotate_command == LEFT ? "Left" : "Right", current_yaw, target_yaw);
    
  //   // 检查旋转是否在允许的范围内
  //   if (angle_diff < 0.0053)  // 约等于0.3度
  //   {
  //     success_msg.data = true;
  //     ROS_INFO("rotate_success");
  //     success_pub.publish(success_msg);  // 发布旋转成功的消息
  //     rotate_command = 0;  // 重置命令，防止进一步旋转
  //   }
  // }
}

// 旋转指令回调函数
void commandCallback(const std_msgs::Int32::ConstPtr& msg)
{
  // rotate_command = msg->data;
  // initial_yaw = current_yaw;  // 记录指令发出时的航向角

  // if (rotate_command == LEFT)
  //   target_yaw = initial_yaw + M_PI_2; // 左转90度的目标航向角
  // else if (rotate_command == RIGHT)
  //   target_yaw = initial_yaw - M_PI_2; // 右转90度的目标航向角

  // // 将目标航向角归一化到0到2π之间
  // target_yaw = fmod(target_yaw, 2 * M_PI);
  // if (target_yaw < 0)
  //   target_yaw += 2 * M_PI;

  // ROS_INFO("Received command to rotate %s: Initial Yaw: %.2f, Target Yaw: %.2f", rotate_command == LEFT ? "Left" : "Right", initial_yaw, target_yaw);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rotation_control_node");
  ros::NodeHandle nh;

  // 设置PID控制器
  pid.setPID(1.0, 0.0, 0.0);

  // 初始化发布器和订阅器
  vel_pub = nh.advertise<geometry_msgs::Twist>("/target_vel_pub", 10);
  success_pub = nh.advertise<std_msgs::Bool>("/rotation_success", 10);
  ros::Subscriber rpy_sub = nh.subscribe("/imu_rpy", 1, rpyCallback);
  ros::Subscriber cmd_sub = nh.subscribe("/rotate_command", 1, commandCallback);

  ros::spin();
  return 0;
}
