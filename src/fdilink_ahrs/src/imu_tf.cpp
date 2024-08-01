#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/Imu.h>

ros::Publisher new_imu_pub;

void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    static tf2_ros::Buffer tf_buffer;
    static tf2_ros::TransformListener tf_listener(tf_buffer);

    try {
        // 获取从/imu到/base_link的坐标系转换
        geometry_msgs::TransformStamped transform = tf_buffer.lookupTransform("base_link", imu_msg->header.frame_id,
                                                                               ros::Time(0), ros::Duration(1.0));

        // 进行坐标系转换
        geometry_msgs::Vector3 acc=imu_msg->linear_acceleration,acc1;
        geometry_msgs::Vector3 ang=imu_msg->angular_velocity,ang1;
        geometry_msgs::Quaternion orie=imu_msg->orientation,orie1;
        tf2::doTransform(acc,acc1,transform);
        tf2::doTransform(ang,ang1,transform);
        tf2::doTransform(orie,orie1,transform);

        // 创建转换后的IMU消息
        sensor_msgs::Imu transformed_imu_msg;
        transformed_imu_msg.header.stamp = ros::Time::now();
        transformed_imu_msg.header.frame_id = "base_link";
        transformed_imu_msg.linear_acceleration = acc1;
        transformed_imu_msg.angular_velocity = ang1;

        new_imu_pub.publish(transformed_imu_msg);
    }
    catch (tf2::TransformException& ex) {
        ROS_WARN("Failed to transform IMU message: %s", ex.what());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_transformer");
    ros::NodeHandle nh;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    new_imu_pub = nh.advertise<sensor_msgs::Imu>("/new_imu", 10);
    ros::Subscriber imu_sub = nh.subscribe("/imu", 10, imuCallback);
    

    ros::spin();

    return 0;
}
