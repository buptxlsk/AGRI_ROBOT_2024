#include <ros/ros.h>
#include <std_msgs/Float64.h>

class DemoYawPublisherNode
{
public:
    DemoYawPublisherNode() : target_yaw_(1.57) // 设置常数目标航向角度 (例如：90度 = 1.57 弧度)
    {
        target_yaw_pub_ = nh_.advertise<std_msgs::Float64>("/target_yaw", 10);
        timer_ = nh_.createTimer(ros::Duration(1.0), &DemoYawPublisherNode::publishTargetYaw, this);
    }

private:
    void publishTargetYaw(const ros::TimerEvent&)
    {
        std_msgs::Float64 message;
        message.data = target_yaw_;
        ROS_INFO("Publishing target yaw: %f", message.data);
        target_yaw_pub_.publish(message);
    }

    ros::NodeHandle nh_;
    ros::Publisher target_yaw_pub_;
    ros::Timer timer_;
    double target_yaw_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "demo_yaw_publisher_node");
    DemoYawPublisherNode node;
    ros::spin();
    return 0;
}
