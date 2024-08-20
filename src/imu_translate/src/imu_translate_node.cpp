#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include "pid.h"

#define LEFT 1
#define RIGHT -1 

ros::Publisher rpy_pub, vel_pub,success_pub;
std_msgs::Bool success_msg;
geometry_msgs::Vector3 rpy_msg;
geometry_msgs::Twist target_vel;
int rotate_direction[6] = {RIGHT, RIGHT, LEFT, LEFT, RIGHT, RIGHT};
int rotate_count = 0, rotate_flag = 0;
double target, cur_yaw, last_yaw,once_yaw;
PID pid;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  ros::NodeHandle nh;
  nh.getParam("/rotate_flag", rotate_flag);  
  // ROS_INFO("rotate_flag = %d", rotate_flag);
  tf2::Quaternion quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
  tf2::Matrix3x3 matrix(quaternion);
  matrix.getRPY(rpy_msg.x, rpy_msg.y, rpy_msg.z);
  if (rpy_msg.z < 0)
  {
    rpy_msg.z = rpy_msg.z + 2 * M_PI;
  }
  nh.setParam("/yaw", rpy_msg.z);
  rpy_pub.publish(rpy_msg);

  // 添加ROS_INFO输出rpy?
  // ROS_INFO("Received IMU data: roll = %f, pitch = %f, yaw = %f", rpy_msg.x, rpy_msg.y, rpy_msg.z);

  if (rotate_flag == 0)
  {
    last_yaw = rpy_msg.z;
  }
  else if (rotate_flag == 1)
  {
    target = rotate_direction[rotate_count] * M_PI_2;
    cur_yaw = rpy_msg.z - last_yaw;
    if (target - cur_yaw > M_PI)
    {
      cur_yaw += 2 * M_PI;
    }
    else if (target - cur_yaw < -M_PI)
    {
      cur_yaw -= 2 * M_PI;
    }
    // ROS_INFO("%lf",target);
    // target_vel.angular.z = pid.calc_output(target, cur_yaw);
    // __LIMIT(target_vel.angular.z, 0.005);
    // vel_pub.publish(target_vel);
    double error = abs(target - cur_yaw);
    ROS_INFO("%lf",error);
    if (abs(error) < 0.001) 
    {
      success_msg.data = true;
      ROS_INFO("rotate_success");
      success_pub.publish(success_msg);  // 发布旋转成功的消xi
      rotate_count++;
      rotate_flag = 0;
      nh.setParam("/rotate_flag", rotate_flag);
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_translate_node");
  pid.setPID(1.0, 0.0, 0.0);
  ros::NodeHandle nh;
  ros::Subscriber imu_sub = nh.subscribe("/IMU_data", 1, imuCallback);
  vel_pub = nh.advertise<geometry_msgs::Twist>("/target_vel_pub", 10);
  rpy_pub = nh.advertise<geometry_msgs::Vector3>("/imu_rpy", 1);
  success_pub = nh.advertise<std_msgs::Bool>("/rotation_success",10);
  ros::spin();
  return 0;
}
