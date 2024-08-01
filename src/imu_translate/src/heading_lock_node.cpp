#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <tf/tf.h>
#include "pid.h"

class HeadingLockNode
{
public:
    HeadingLockNode()
    {
        imu_sub_ = nh_.subscribe("/imu/data", 10, &HeadingLockNode::imuCallback, this);
        target_yaw_sub_ = nh_.subscribe("/target_yaw", 10, &HeadingLockNode::targetYawCallback, this);
        control_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        pid_.setPID(1.0, 0.0, 0.0);
    }

    void spin()
    {
        ros::Rate rate(10); // 10 Hz

        while (ros::ok())
        {
            ros::spinOnce();
            controlLoop();
            rate.sleep();
        }
    }

private:
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
    {
        tf::Quaternion quaternion(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w
        );
        tf::Matrix3x3 matrix(quaternion);
        double roll, pitch, yaw;
        matrix.getRPY(roll, pitch, yaw);
        if (yaw < 0)
        {
            yaw += 2 * M_PI;
        }
        current_yaw_ = yaw;
    }

    void targetYawCallback(const std_msgs::Float64::ConstPtr& msg)
    {
        target_yaw_ = msg->data;
        ROS_INFO("Received target yaw: %f", target_yaw_);
    }

    void controlLoop()
    {
        double control_signal = pid_.calc_output(target_yaw_, current_yaw_);
        geometry_msgs::Twist control_msg;
        control_msg.angular.z = control_signal;
        control_pub_.publish(control_msg);
        ROS_INFO("Publishing control signal: angular.z = %f", control_msg.angular.z);
    }

    ros::NodeHandle nh_;
    ros::Subscriber imu_sub_;
    ros::Subscriber target_yaw_sub_;
    ros::Publisher control_pub_;

    double target_yaw_;
    double current_yaw_;
    PID pid_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "heading_lock_node");
    HeadingLockNode node;
    node.spin();
    return 0;
}
