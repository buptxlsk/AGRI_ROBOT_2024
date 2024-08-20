#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <string.h>

#define EDGE_DISTANCE 0.37
#define VASE 1
#define VASE_GREEN 2
#define VASE_BLUE 3
#define VASE_RED 4

using namespace std;
class DIS
{
    private:
    ros::NodeHandle nh;
    ros::Subscriber distance_sub;
    ros::Publisher vase_detect_pub;
    ros::Subscriber vase_sub;
    string topic_vase_info,topic_vase_detect,topic_distance_sub;
    float distance_min,distance_round,now_dis,last_distance,min_dis=1;
    std_msgs::Int32 vase_condition;
    double distance_edge=0.37;
    bool is_vase=false;
    int count_vase=0;
    int curField,last_Field=0;
    ros::Time now_time,begin_time;
    public:
    DIS(ros::NodeHandle &handle)
    {
        nh=handle;
        nh.getParam("distance_sub",topic_distance_sub);
        nh.getParam("vase_detect",topic_vase_detect);
        nh.getParam("vase_info",topic_vase_info);
        nh.getParam("/current_field",curField);
        // nh.getParam("distance_egde",distance_edge);
        distance_sub = nh.subscribe(topic_distance_sub,10,&DIS::distance_reader_callback,this);
        vase_sub = nh.subscribe(topic_vase_info,10,&DIS::vase_get_callback,this);
        vase_detect_pub = nh.advertise<std_msgs::Int32>(topic_vase_detect,10);
        distance_edge = EDGE_DISTANCE;
    }
    void distance_reader_callback(const std_msgs::Float32::ConstPtr &msg)
    {
        nh.getParam("/current_field",curField);
        if(last_Field!=curField)
        {
            last_Field = curField;
            distance_edge=EDGE_DISTANCE;
        }
        
        if(msg->data <= distance_edge){
            vase_condition.data = VASE;
            // if (min_dis>now_dis) {
            //     min_dis = now_dis;
            // }
            if (is_vase == false) {
                is_vase = true;
                // ros::Duration(0.5).sleep();
                vase_detect_pub.publish(vase_condition);
                ROS_INFO("send_vase");
                distance_edge = (msg->data+EDGE_DISTANCE)/2+0.03;
                if(distance_edge>=EDGE_DISTANCE+0.05)distance_edge = EDGE_DISTANCE;
            }
            // if (now_dis> min_dis+0.005&&is_vase == false) {
            // }
        }
        if(msg->data>= distance_edge){
            is_vase = false;
            min_dis = 1;

        }
        last_distance = msg->data;
    }
    void vase_get_callback(const std_msgs::Int32::ConstPtr &msg)
    {
        if (is_vase!=true) {
            
        }
        else if(msg->data==VASE_GREEN){
            vase_condition.data = VASE_GREEN;
            vase_detect_pub.publish(vase_condition);
            ROS_INFO("send_vase_green");

        }
        else if (msg->data==VASE_BLUE) {
            vase_condition.data = VASE_BLUE;            
            vase_detect_pub.publish(vase_condition);
            ROS_INFO("send_vase_blue");

        }
        else if (msg->data==VASE_RED) {
            vase_condition.data = VASE_RED;
            vase_detect_pub.publish(vase_condition);
            ROS_INFO("send_vase_red");
        }
    }

};
int main(int argc,char **argv)
{
    ros::init(argc,argv,"dis_judge");
    ros::NodeHandle nh;
    DIS dis(nh);
    ros::Duration(3.0).sleep();
    ros::Rate loop_rate(10);
    ROS_INFO("judge start");
    while (ros::ok) {
        ros::spinOnce();
        loop_rate.sleep();
        
    }
}
