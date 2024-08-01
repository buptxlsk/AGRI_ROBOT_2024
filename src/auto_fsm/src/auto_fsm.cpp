#include <iterator>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/builtin_double.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/UInt16MultiArray.h>
#include "pid.h"

#define WHITECROSS 0  //白十字
#define VASE 1  //花瓶
#define VASE_GREEN 2
#define VASE_BLUE 3
#define VASE_RED 4
#define DISTANCE_MIN 0.2

double last_yaw;
int rotate_flag = 0;
double default_vel = 0.2;
double default_rotate_vel = 0.7;

enum STATE
{
    STOP = 0,
    MOVE,
    ROTATE,
    MOVE_IN_CORNER,
    STOP_TO_WATER_LEFT,
    STOP_TO_WATER_RIGHT,
    START
};

// enum DROUGHT_LEVEL
//{
//    SLIGHT = 1,
//    NORMAL,
//    SERIOUS
//};

#define SLIGHT 1
#define NORMAL 2
#define SERIOUS 3

int vase_list[3][6]={
    //{SLIGHT, SLIGHT, NORMAL, NORMAL, SERIOUS, SERIOUS},
    //{SLIGHT, SLIGHT, NORMAL, NORMAL, SERIOUS, SERIOUS},
    //{SLIGHT, SLIGHT, NORMAL, NORMAL, SERIOUS, SERIOUS},
     {1,2,3,1,2,3},
     {1,1,2,3,2,3},
     {2,3,1,1,3,2}
};
int curField = 0,cnt_left = 0, cnt_right = 0;
int curVaseColor = 0;

class FSM
{
private:
    STATE current_state;
    ros::NodeHandle nh;
    ros::Subscriber cross_vision_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber distance_sub_left;//dt35订阅left
    ros::Subscriber distance_sub_right;//dt35订阅right
    ros::Publisher target_vel_pub;
    ros::Subscriber vase_vision_sub_left;
    ros::Subscriber vase_vision_sub_right;
    ros::Subscriber get_drought_level_sub;
    double target_vel;     //目标速度值，linear.x 
    geometry_msgs::Twist target_vel_twist;
    int rotate_count;  //用于摆脱十字识别
    double target_rotate_vel;
    double yaw; //用于走直线
    bool start_flag;

    bool start_flag2;

    PID pid;
public:
    FSM(ros::NodeHandle &handle)
    {
        ROS_INFO("FSM init");
        nh = handle;
        current_state = START;
        start_flag = false; start_flag2 = false;
        rotate_count = 0;
        cross_vision_sub = nh.subscribe("/cross_vision", 10, &FSM::cross_vision_callback, this);
        vase_vision_sub_left = nh.subscribe("/vase_detect_left", 10, &FSM::vase_vision_left_callback, this);
        vase_vision_sub_right = nh.subscribe("/vase_detect_right", 10, &FSM::vase_vision_right_callback, this);
        target_vel_pub = nh.advertise<geometry_msgs::Twist>("/target_vel_pub", 10);
        get_drought_level_sub = nh.subscribe("/tcp_data", 10, &FSM::get_drought_level_callback, this);
        distance_sub_left = nh.subscribe("/distance_data_left",10,&FSM::distance_reader_left_callback,this);
        distance_sub_right = nh.subscribe("/distance_data_right",10,&FSM::distance_reader_right_callback,this);

        target_vel = default_vel;
        target_rotate_vel = default_rotate_vel;
        nh.setParam("/current_state", current_state);
        nh.setParam("/current_field", curField);
        pid.setPID(1.0, 0.0, 0.0);
    }

    void cross_vision_callback(const std_msgs::Int32::ConstPtr &msg)
    {
        if (msg->data == WHITECROSS)
        {
            FSM_next_state(ROTATE);
            cross_vision_sub.shutdown();
            ROS_INFO("WHITECROSS");
            rotate_flag = 1;
            target_vel_twist.linear.x = 0;
            target_vel_twist.linear.z = 2;
            target_vel_twist.angular.z = 0;
            target_vel_pub.publish(target_vel_twist);
            target_vel_twist.linear.z = 0;
            ros::Duration(0.2).sleep();
            nh.setParam("/rotate_flag", rotate_flag);
        }
        
    }

    void vase_vision_left_callback(const std_msgs::Int32::ConstPtr &msg)
    {

        if (msg->data == VASE_BLUE)
        {
            FSM_next_state(STOP_TO_WATER_LEFT);
            curVaseColor = VASE_BLUE;
            ROS_INFO("LEFT_VASE_BLUE_DETECT");
        }
        else if (msg->data == VASE_GREEN)
        {
            FSM_next_state(STOP_TO_WATER_LEFT);
            curVaseColor = VASE_GREEN;
            ROS_INFO("LEFT_VASE_GREEN_DETECT");
        }
        else if (msg->data == VASE_RED)
        {
            FSM_next_state(STOP_TO_WATER_LEFT);
            curVaseColor = VASE_RED;
            ROS_INFO("LEFT_VASE_RED_DETECT");
        }
    }

    void vase_vision_right_callback(const std_msgs::Int32::ConstPtr &msg)
    {
        if (msg->data == VASE_BLUE)
        {
            FSM_next_state(STOP_TO_WATER_LEFT);
            curVaseColor = VASE_BLUE;
            ROS_INFO("RIGHT_VASE_BLUE_DETECT");
        }
        else if (msg->data == VASE_GREEN)
        {
            FSM_next_state(STOP_TO_WATER_LEFT);
            curVaseColor = VASE_GREEN;
            ROS_INFO("RIGHT_VASE_GREEN_DETECT");
        }
        else if (msg->data == VASE_RED)
        {
            FSM_next_state(STOP_TO_WATER_LEFT);
            curVaseColor = VASE_RED;
            ROS_INFO("RIGHT_VASE_RED_DETECT");
        }
    }

    void distance_reader_left_callback(const std_msgs::Float32::ConstPtr &msg)
    {
        if(msg->data >=DISTANCE_MIN&&msg->data <= (DISTANCE_MIN+0.2)){
            FSM_next_state(STOP_TO_WATER_LEFT);
            ROS_INFO("VASE_DETECT");
        }
    }
        void distance_reader_right_callback(const std_msgs::Float32::ConstPtr &msg)
    {
        if(msg->data >=DISTANCE_MIN&&msg->data <= (DISTANCE_MIN+0.2)){
            FSM_next_state(STOP_TO_WATER_RIGHT);
            ROS_INFO("VASE_DETECT");
        }
    }
    void get_drought_level_callback(const std_msgs::UInt8MultiArray::ConstPtr &msg)
    {
        if (msg->data[0] == 252 && msg->data[19] == 254)
        {
            for (int i = 0; i < 6; i++)
            {
                vase_list[0][i] = msg->data[i + 1];
            }
            for (int i = 0; i < 6; i++)
            {
                vase_list[1][i] = msg->data[i + 7];
            }
            for (int i = 0; i < 6; i++)
            {
                vase_list[2][i] = msg->data[i + 13];
            }
            start_flag = true;  
        }
    }
    
    void FSM_next_state(STATE next_state)
    {
        if (current_state == next_state)
        {
            return;
        }
        else
        {
            switch (next_state)
            {
                case STOP:
                {
                    ROS_INFO("SWITCH_TO_STOP");
                    break;
                }
                case MOVE:
                {
                    ROS_INFO("SWITCH_TO_MOVE");
                    if(current_state == ROTATE)
                    {
                        target_vel_twist.linear.x = 0.1;
                        target_vel_twist.linear.z = 1;
                        target_vel_pub.publish(target_vel_twist);
                        ros::Duration(0.5).sleep();
                        cross_vision_sub = nh.subscribe("/cross_vision", 10, &FSM::cross_vision_callback, this);
                    }
                    nh.getParam("/yaw", last_yaw);
                    break;
                }
                case ROTATE:
                {
                    rotate_count += 1;
                    ROS_INFO("SWITCH_TO_ROTATE");
                    break;
                }
                case STOP_TO_WATER_LEFT:
                {   
                    ROS_INFO("SWITCH_TO_STOP_TO_WATER_LEFT");
                    target_vel_twist.linear.x = 0;
                    target_vel_twist.linear.z = 0;
                    target_vel_twist.angular.z = 0;
                    target_vel_pub.publish(target_vel_twist);
                    break;
                }
                case STOP_TO_WATER_RIGHT:
                {   
                    ROS_INFO("SWITCH_TO_STOP_TO_WATER_RIGHT");
                    target_vel_twist.linear.x = 0;
                    target_vel_twist.linear.z = 0;
                    target_vel_twist.angular.z = 0;
                    target_vel_pub.publish(target_vel_twist);
                    break;
                }
                case MOVE_IN_CORNER:
                {
                    ROS_INFO("SWITCH_TO_MOVE_IN_CORNER");
                    if(current_state == ROTATE)
                    {
                        target_vel_twist.linear.x = 0.1;
                        target_vel_pub.publish(target_vel_twist);
                        ros::Duration(0.5).sleep();
                        cross_vision_sub = nh.subscribe("/cross_vision", 10, &FSM::cross_vision_callback, this);
                    }
                    nh.getParam("/yaw", last_yaw);
                    break;
                }
            }
            current_state = next_state;
            nh.setParam("/current_state", current_state);
        }
    }

    void FSM_exec()
    {
        nh.getParam("/rotate_flag", rotate_flag);
        if (rotate_flag == 1) FSM_next_state(ROTATE);
        switch (current_state)
        {
            case STOP:
                {
                    ROS_INFO("STOP");
                    target_vel_twist.linear.x = 0;
                    target_vel_twist.angular.z = 0;
                    target_vel_pub.publish(target_vel_twist);
                    break;
                }
            case MOVE:
                {
                    nh.getParam("/yaw", yaw);
                    target_vel_twist.linear.x = 0.1;
                    target_vel_twist.angular.z = pid.calc_output(last_yaw, yaw, true);
                    __LIMIT(target_vel_twist.angular.z, 0.05);
                    target_vel_pub.publish(target_vel_twist);
                    break;
                }
            case ROTATE:
                {
                    nh.getParam("/rotate_flag", rotate_flag);
                    if (rotate_flag == 0) {
                        if(rotate_count % 2 == 0){
                            FSM_next_state(MOVE);
                            curField++;
                            nh.setParam("/current_field", curField);
                            cnt_left=0;
                            cnt_right=0;
                        }
                        else if(rotate_count % 2 == 1){
                            FSM_next_state(MOVE_IN_CORNER);
                        }
                    }
                    ros::spinOnce();
                    break;
                }
            case STOP_TO_WATER_LEFT://待增加：伸出机械臂
                {
                    ROS_INFO("STOP_TO_WATER_LEFT");
                    target_vel_twist.linear.x = 0;
                    if (curField<3) target_vel_twist.linear.y = vase_list[curField][cnt_left];
                    else target_vel_twist.linear.y = curVaseColor - 1;
                    target_vel_twist.angular.x = 1;
                    target_vel_twist.angular.z = 0;
                    target_vel_pub.publish(target_vel_twist);
                    target_vel_twist.linear.y = 0;
                    cnt_left++;
                    ros::Duration(5).sleep();
                    FSM_next_state(MOVE);
                    break;
                }
            case STOP_TO_WATER_RIGHT://待增加：伸出机械臂
                {
                    ROS_INFO("STOP_TO_WATER_RIGHT");
                    target_vel_twist.linear.x = 0;
                    if (curField<3) target_vel_twist.linear.y = vase_list[curField][cnt_right];
                    else target_vel_twist.linear.y = curVaseColor - 1;
                    target_vel_twist.angular.x = 2;
                    target_vel_twist.angular.z = 0;
                    target_vel_pub.publish(target_vel_twist);
                    target_vel_twist.linear.y = 0;
                    cnt_right++;
                    ros::Duration(5).sleep();
                    FSM_next_state(MOVE);
                    break;
                }
            case MOVE_IN_CORNER:
                {
                    ROS_INFO("MOVE_IN_CORNER at speed %f", target_vel);
                    nh.getParam("/yaw", yaw);
                    target_vel_twist.linear.x = 0.1;
                    ROS_INFO("%f", yaw - last_yaw);
                    target_vel_twist.angular.z = pid.calc_output(last_yaw, yaw, true);
                    __LIMIT(target_vel_twist.angular.z, 0.05);
                    target_vel_pub.publish(target_vel_twist);
                    break;
                }
            case START:
                {
                    ROS_INFO("START");
                    target_vel_twist.linear.x = 0;
                    target_vel_twist.angular.z = 0;
                    target_vel_twist.linear.z = 1;
                    target_vel_pub.publish(target_vel_twist);
                    target_vel_twist.linear.z = 0;

                    start_flag=true;

                    if (start_flag)
                    {
                        get_drought_level_sub.shutdown();
                        FSM_next_state(MOVE);
                    }
                }
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "auto_fsm");
    ros::NodeHandle nh;
    FSM fsm(nh);
    ros::Duration(6).sleep();
    nh.setParam("/rotate_flag", rotate_flag);
    ros::Rate loop_rate(10);
    nh.getParam("/yaw", last_yaw);
    ROS_INFO("%f", last_yaw);
    ROS_INFO("FSM start");
    
    while (ros::ok())
    {
        ros::spinOnce();
        fsm.FSM_exec();
        loop_rate.sleep();
    }
    return 0;
}
