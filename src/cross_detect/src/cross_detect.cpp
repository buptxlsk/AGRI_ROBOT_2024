#include <ros/ros.h>

#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <string.h>
#include <unistd.h>

#define WHITECROSS 0
#define GO_FORWARD 1
#define GO_BEHIND -1
using namespace std;
class CROSS
{
private:
    ros::Subscriber grayscale_info_ahead;
    ros::Subscriber grayscale_info_behind;
    ros::Publisher cross_vision_pub;
    ros::Subscriber rotate_success;
    int ahead_judge=0,behind_judge=0;
    string topic_ahead,topic_behind;
    std_msgs::Int32 cross_detect;
    bool con_ahead=false,con_behind=false,ahead_last=false,behind_last=false;
    int count_ahead=0,count_behind=0,judge=0;
public:
    CROSS(ros::NodeHandle &handle)
    {
        ros::NodeHandle nh = handle;
        nh.getParam("gray_topic_ahead",topic_ahead);
        nh.getParam("gray_topic_behind",topic_behind);
        rotate_success = nh.subscribe("/rotation_success",10,&CROSS::rotate_callback,this);
        grayscale_info_ahead = nh.subscribe(topic_ahead,10,&CROSS::cross_info_callback_ahead,this);
        grayscale_info_behind = nh.subscribe(topic_behind,10,&CROSS::cross_info_callback_behind,this);
        cross_vision_pub = nh.advertise<std_msgs::Int32>("/cross_vision",10);
    }
    void rotate_callback(const std_msgs::Bool::ConstPtr &msg)
    {
        if(msg->data==true)
        {
            ahead_judge=0;
            behind_judge=0;
            judge=0;
            ros::Duration(10).sleep();
            ROS_INFO("judge_clear");
        }
        else
        {
            ahead_judge=0;
            behind_judge=0;
            judge=0;
        }
    }
    void cross_info_callback_ahead(const std_msgs::String::ConstPtr &msg)
    {
        string info = msg->data;
        string s = "1";
        for(int i = 1;i<16;i++){
            if (info.compare(i,1,s)==0) {
                count_ahead++;
            }

        }
        if (count_ahead>=5) {
            con_ahead = true;
            if (ahead_last == false){
                ahead_judge++;
                ROS_INFO("ahead_detect");
            }
        }
        else con_ahead = false;
        count_ahead = 0;
        ahead_last = con_ahead;
    }

    void cross_info_callback_behind(const std_msgs::String::ConstPtr &msg)
    {
        string info = msg->data;

        for(int i = 1;i<16;i++){
            if (info.compare(i,1,"1")==0) {
                count_behind++;   
            }
        }
        if (count_behind>=5) {
            con_behind = true;
            if (behind_last == false){
                behind_judge++;
                ROS_INFO("behind_detect");
            }
        }
        else con_behind = false;
        behind_last = con_behind;
        count_behind = 0;

    }
    void cross_pub(){
        if (ahead_judge!=0)
        {
            cross_detect.data = GO_FORWARD;
            if(judge == 0){
                cross_vision_pub.publish(cross_detect);
                judge++;
            }

            if(behind_judge!=0)
            {
                cross_detect.data = GO_BEHIND;
                if(judge == 1){
                    cross_vision_pub.publish(cross_detect);
                    judge++;
                }

                if(con_behind==false&&judge==2){
                    cross_detect.data = WHITECROSS;
                    ros::Duration(0.1).sleep();
                    cross_vision_pub.publish(cross_detect);
                    judge++;
                }
            }
        }

    }
};
int main(int argc, char** argv){
    ros::init(argc, argv, "cross_detect");
    ros::NodeHandle nh;
    CROSS cross(nh);
    ROS_INFO("cross_start");
    ros::Rate loop_rate(10);
    while(ros::ok()){
        ros::spinOnce();
        cross.cross_pub();
        loop_rate.sleep();
    }
}

