#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <iostream>
#include <unistd.h>


using namespace std;
class CROSS
{
private:
    ros::Subscriber grayscale_info_ahead,grayscale_info_behind;//data[0]无效,右起为第一位，1～15位
    ros::Publisher cross_version_pub;
    int judge;//判断遇线光感
    string gray_topic_ahead,gray_topic_behind;
    ros::std_msgs::Int32 cross_detect;
    bool last_con_ahead,last_con_behind,current_con_ahead,current_con_behind;
    int count_ahead=0,count_behind=0;
public:
    CROSS(ros::NodeHandle nh)
    {
        nh.getParam("gray_topic_ahead",gray_topic_ahead);
        nh.getParam("gray_topic_behind",gray_topic_behind);
        grayscale_info_ahead = nh.Subscribe(gray_topic_ahead,10,cross_info_callback_ahead,this);
        grayscale_info_behind = nh.Subscribe(gray_topic_behind,10,cross_info_callback_behind,this);
        cross_version_pub = nh.advertise<std_msgs::Int32>("/cross_version",10);
    }
    void cross_info_callback(const std_msgs::String::ConstPtr &msg)
    {
        string *info = msg->data;
        for(int i = 1;i<16;i++){
            if (info[i]=='1') {
                if (i<7&&i>9) count_ahead++;            
            }
        }
        if (count_ahead>=5) {
            
        }
    }
};


