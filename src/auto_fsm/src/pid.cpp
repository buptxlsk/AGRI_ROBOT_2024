#include "pid.h"
#include <ros/ros.h>

template<typename T>void __LIMIT(T &a,const T &b)
{
    if (a>b) a=b;
    else if (a<-b) a=-b;
}

PID::PID()
{
    kp=ki=kd=int_sum=0;
    last_error=last_delta_error=0;
    int_max=ctrl_max=200;
}

double PID::calc_output(double target, double actual, bool is_angle)
{
    double error;
    double delta_error;
    double result;

    error = target - actual;
    if (is_angle)
    {
        if (error > M_PI) error -= 2 * M_PI;
        else if (error < -M_PI) error += 2 * M_PI;
    }
    delta_error = error - last_error;

    delta_error *= 0.384f;
    delta_error += last_delta_error * 0.615f; //低通滤波

    last_error = error;

    int_sum += error;

    __LIMIT(int_sum, int_max); // 积分限幅
    last_delta_error = delta_error;

    result = error * kp + delta_error * kd + int_sum * ki;
    __LIMIT(result, ctrl_max);
    return result;
}

void PID::setPID(double p, double i, double d)
{
    kp=p,ki=i,kd=d;
}
