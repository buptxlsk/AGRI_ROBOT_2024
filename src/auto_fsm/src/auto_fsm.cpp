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

#define WHITECROSS 0  
#define VASE 1  
#define VASE_GREEN 2
#define VASE_BLUE 3
#define VASE_RED 4
#define DISTANCE_MIN 0.2

#define LEFT 1
#define RIGHT -1 

#define GO_FORWARD 1
#define GO_BEHIND -1
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
    START,
    CROSS_AHEAD,
    CROSS_BEHIND
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
     {3,2,1,3,2,1},
     {1,2,1,2,1,2}
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
    ros::Publisher target_vel_pub;
    ros::Publisher rotate_command_pub;
    ros::Publisher rotate_reset_pub;
    ros::Subscriber vase_vision_sub_left;
    ros::Subscriber vase_vision_sub_right;
    ros::Subscriber get_drought_level_sub;
    ros::Subscriber distance_info_sub_left;
    ros::Subscriber distance_info_sub_right;
    ros::Subscriber rotate_success;
    double target_vel;     
    geometry_msgs::Twist target_vel_twist;
    std_msgs::Int32 rotate_command;
    std_msgs::Bool rotate_reset;
    int rotate_count;  
    int move_count=0;
    double target_rotate_vel;
    double yaw; 
    bool start_flag;
    double move_speed = 0.005;
    bool start_flag2;
    float left_distance,right_distance;
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
        distance_info_sub_left = nh.subscribe("/distance_data_left",10,&FSM::distance_info_left_callback,this);
        distance_info_sub_right = nh.subscribe("/distance_data_right",10,&FSM::distance_info_right_callback,this);
        rotate_success = nh.subscribe("/rotation_success",10,&FSM::rotate_callback,this);
        target_vel_pub = nh.advertise<geometry_msgs::Twist>("/target_vel_pub", 10);
        rotate_command_pub = nh.advertise<std_msgs::Int32>("/rotate_command",10);
        rotate_reset_pub = nh.advertise<std_msgs::Bool>("/rotation_success",10);
        get_drought_level_sub = nh.subscribe("/tcp_data", 10, &FSM::get_drought_level_callback, this);
        // cross_vision_sub.shutdown();
        target_vel = default_vel;
        target_rotate_vel = default_rotate_vel;
        nh.setParam("/current_state", current_state);
        nh.setParam("/current_field", curField);
        nh.setParam("move_speed",move_speed);
        pid.setPID(1.0, 0.0, 0.0);
    }

    void cross_vision_callback(const std_msgs::Int32::ConstPtr &msg)
    {
        if (msg->data == WHITECROSS)
        {
            move_count=1;
            rotate_flag = 1;
            nh.setParam("/rotate_flag",rotate_flag);
            // cross_vision_sub.shutdown();
            ROS_INFO("WHITECROSS");
            target_vel_twist.linear.x = 0;
            // target_vel_twist.linear.z = 2;
            target_vel_twist.angular.z = 0;
            target_vel_pub.publish(target_vel_twist);
            // nh.getParam("/yaw", last_yaw);

            // target_vel_twist.linear.z = 0;
            ros::Duration(0.5).sleep();
            FSM_next_state(ROTATE);
            nh.setParam("/rotate_flag", rotate_flag);
            
        }
        if (msg->data==GO_FORWARD)
        {
            // nh.getParam("/yaw", last_yaw);
            target_vel_twist.linear.x = 0.002;
            target_vel_twist.angular.z = pid.calc_output(last_yaw, yaw, true);
            __LIMIT(target_vel_twist.angular.z, 0.005);
            ROS_INFO("GO_FORWARD");
            target_vel_pub.publish(target_vel_twist);
            if(curField==3)
            {
                ROS_INFO("STOP_READY");
                ros::Duration(5.0).sleep();
                
                FSM_next_state(STOP);
            }
            else  FSM_next_state(CROSS_AHEAD);

        }
        if(msg->data==GO_BEHIND)
        {
            ROS_INFO("GO_BEHIND");
            target_vel_twist.linear.x=0;
            target_vel_twist.linear.z=0;
            target_vel_pub.publish(target_vel_twist);
            ros::Duration(0.4).sleep();
            target_vel_twist.linear.x = -0.002;
            target_vel_twist.angular.z = pid.calc_output(last_yaw,yaw,true);
            __LIMIT(target_vel_twist.angular.z, 0.005);
            target_vel_pub.publish(target_vel_twist);
            FSM_next_state(CROSS_BEHIND);
        }
    }

    void vase_vision_left_callback(const std_msgs::Int32::ConstPtr &msg)
    {
        if(move_count==0)
        {
            if (msg->data == VASE) {
                FSM_next_state(STOP_TO_WATER_LEFT);
                ROS_INFO("LEFT_VASE_DETECT");
            }
            else if (msg->data == VASE_BLUE)
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
    }

    void vase_vision_right_callback(const std_msgs::Int32::ConstPtr &msg)
    {
        if(move_count==0)
        {
            if (msg->data == VASE) {
                FSM_next_state(STOP_TO_WATER_RIGHT);
                ROS_INFO("VASE_DETECT");
            }
            else if (msg->data == VASE_BLUE)
            {
                FSM_next_state(STOP_TO_WATER_RIGHT);
                curVaseColor = VASE_BLUE;
                ROS_INFO("RIGHT_VASE_BLUE_DETECT");
            }
            else if (msg->data == VASE_GREEN)
            {
                FSM_next_state(STOP_TO_WATER_RIGHT);
                curVaseColor = VASE_GREEN;
                ROS_INFO("RIGHT_VASE_GREEN_DETECT");
            }
            else if (msg->data == VASE_RED)
            {
                FSM_next_state(STOP_TO_WATER_RIGHT);
                curVaseColor = VASE_RED;
                ROS_INFO("RIGHT_VASE_RED_DETECT");
            }
        }
    }
    void distance_info_left_callback(const std_msgs::Float32::ConstPtr &msg)
    {
        left_distance=msg->data;
    }
    void distance_info_right_callback(const std_msgs::Float32::ConstPtr &msg)
    {
        right_distance=msg->data;
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
    
    void rotate_callback(const std_msgs::Bool::ConstPtr &msg)
    {
        if(msg->data==true){
            rotate_flag=0;
            nh.setParam("/rotate_flag",rotate_flag);
            ROS_INFO("rotate_success");
        }
    }

    void FSM_next_state(STATE next_state)
    {
        nh.getParam("move_speed",move_speed);
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
                        target_vel_twist.linear.x = 0;
                        // target_vel_twist.linear.z = 1;
                        target_vel_twist.angular.z = 0;
                        target_vel_pub.publish(target_vel_twist);
                        ros::Duration(0.5).sleep();
                        if(curField==3)target_vel_twist.angular.x = 3;
                        target_vel_pub.publish(target_vel_twist);
                        ros::Duration(0.5).sleep();
                        cross_vision_sub = nh.subscribe("/cross_vision", 10, &FSM::cross_vision_callback, this);
                    }
                    // cross_vision_sub.shutdown();
                    // if(!((cnt_right>=1&&cnt_right<5)||(cnt_left>=1&&cnt_left<5)))
                    // {
                    //     nh.getParam("/yaw", last_yaw);
                    //     ROS_INFO("get last_yaw");
                    //     ROS_INFO("%f", last_yaw);

                    // }
                    if(cnt_left <= 0&&cnt_right <= 0)
                    {
                        nh.getParam("/yaw", last_yaw);
                        ROS_INFO("get_yaw");
                    }


                    
                    
                    if(cnt_left>=5||cnt_right>=5)
                    {
                        ROS_INFO("start_cross");
                        cross_vision_sub = nh.subscribe("/cross_vision", 10, &FSM::cross_vision_callback, this);
                        rotate_reset.data = false;
                        rotate_reset_pub.publish(rotate_reset);
                    } 
                    else if(curField==3&&(cnt_left>=3||cnt_right>=3))
                    {
                        ROS_INFO("finish_cross");
                        cross_vision_sub = nh.subscribe("/cross_vision", 10, &FSM::cross_vision_callback, this);
                        rotate_reset.data = false;
                        rotate_reset_pub.publish(rotate_reset);
                        
                    }
                    else
                    {
                        cross_vision_sub.shutdown();                        
                    }
                    break;
                }
                case CROSS_AHEAD:
                {
                    ROS_INFO("AHEAD_DETECT");
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
                    if(move_count==0){
                        if(curField==1)ros::Duration(2.0).sleep();
                        else ros::Duration(0.5).sleep();
                        ROS_INFO("SWITCH_TO_STOP_TO_WATER_LEFT");
                        target_vel_twist.linear.x = 0;
                        // target_vel_twist.linear.z = 0;
                        target_vel_twist.angular.z = 0;
                        target_vel_pub.publish(target_vel_twist);
                    }
                                            

                    break;
                }
                case STOP_TO_WATER_RIGHT:
                {   
                    if(move_count==0)
                    {
                        if(curField==1)ros::Duration(2.0).sleep();
                        else ros::Duration(0.5).sleep();
                        ROS_INFO("SWITCH_TO_STOP_TO_WATER_RIGHT");
                        target_vel_twist.linear.x = 0;
                        // target_vel_twist.linear.z = 0;
                        target_vel_twist.angular.z = 0;
                        target_vel_pub.publish(target_vel_twist);
                    }

                    break;
                }
                case MOVE_IN_CORNER:
                {
                    ROS_INFO("SWITCH_TO_MOVE_IN_CORNER");
                    if(current_state == ROTATE)
                    {
                        target_vel_twist.linear.x = move_speed;
                        target_vel_twist.angular.z =0;
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
                    target_vel_twist.angular.x = 4;
                    target_vel_pub.publish(target_vel_twist);
                    while(1){}
                    break;
                }
            case MOVE:
                {
                    nh.getParam("/yaw", yaw);
                    target_vel_twist.linear.x = move_speed;
                    target_vel_twist.angular.z = pid.calc_output(last_yaw, yaw, true);
                    __LIMIT(target_vel_twist.angular.z, 0.01);
                    target_vel_pub.publish(target_vel_twist);
                    break;
                }
            case ROTATE:
                {
                    nh.getParam("/rotate_flag", rotate_flag);
                    if (rotate_flag == 0) {

                        if(rotate_count % 2 == 0){
                            curField++;
                            cnt_left=0;
                            cnt_right=0;
                            FSM_next_state(MOVE);
                            nh.setParam("/current_field", curField);

                            move_count=0;
                        }
                        else if(rotate_count % 2 == 1){
                            FSM_next_state(MOVE_IN_CORNER);
                        }
                    }
                    else
                    {
                        if(curField%2 == 1)
                        {
                            rotate_command.data = LEFT;
                            // ROS_INFO("TURN_LEFT");     
                            target_vel_twist.linear.x=0;
                            // target_vel_twist.linear.z=0;
                            target_vel_twist.angular.z=0.003;
                            target_vel_pub.publish(target_vel_twist);
                            // rotate_command_pub.publish(rotate_command);
                        }
                        else
                        {
                            rotate_command.data = RIGHT;
                            // ROS_INFO("TURN_RIGHT");
                            target_vel_twist.linear.x=0;
                            // target_vel_twist.linear.z=0;
                            target_vel_twist.angular.z=-0.003;
                            target_vel_pub.publish(target_vel_twist);
                            // rotate_command_pub.publish(rotate_command);
                        }
                    }

                    ros::spinOnce();
                    break;
                }
            case CROSS_AHEAD:
                {
                    break;
                }
            case STOP_TO_WATER_LEFT:
                {

                        ROS_INFO("STOP_TO_WATER_LEFT");
                        target_vel_twist.linear.x = 0;
                        if (curField<3&&cnt_left<=5) target_vel_twist.linear.y = vase_list[curField][cnt_left];
                        else target_vel_twist.linear.y = curVaseColor - 1;
                        target_vel_twist.angular.y = left_distance;
                        target_vel_twist.angular.x = 1;
                        target_vel_twist.angular.z = 0;
                        target_vel_pub.publish(target_vel_twist);
                        ros::Duration(0.1).sleep();
                        target_vel_pub.publish(target_vel_twist);
                        target_vel_twist.linear.y = 0;
                        target_vel_twist.angular.x = 0;
                        target_vel_twist.angular.y = 0;
                        if(cnt_left>cnt_right)cnt_right = cnt_left;
                        cnt_left++;
                        ros::Duration(5).sleep();
                        FSM_next_state(MOVE);
                    

                    break;
                }
            case STOP_TO_WATER_RIGHT:
                {

                    ROS_INFO("STOP_TO_WATER_RIGHT");
                    target_vel_twist.linear.x = 0;
                    if (curField<3&&cnt_right<=5) target_vel_twist.linear.y = vase_list[curField][cnt_right];
                    else target_vel_twist.linear.y = curVaseColor - 1;
                    target_vel_twist.angular.y = right_distance;
                    // if(curField<3)target_vel_twist.angular.x = 3;
                    // else 
                    target_vel_twist.angular.x = 2;
                    target_vel_twist.angular.z = 0;
                    target_vel_pub.publish(target_vel_twist);
                    ros::Duration(0.1).sleep();
                    target_vel_pub.publish(target_vel_twist);
                    target_vel_twist.linear.y = 0;
                    target_vel_twist.angular.x = 0;
                    target_vel_twist.angular.y = 0;
                    if(cnt_right>cnt_left)cnt_left = cnt_right;
                    cnt_right++;
                    ros::Duration(5).sleep();
                    FSM_next_state(MOVE);
                
                    break;
                }
            case MOVE_IN_CORNER:
                {
                    move_count=1;
                    nh.getParam("/yaw", yaw);
                    target_vel_twist.linear.x = move_speed;
                    target_vel_twist.angular.z = pid.calc_output(last_yaw, yaw, true);
                    __LIMIT(target_vel_twist.angular.z, 0.01);
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
