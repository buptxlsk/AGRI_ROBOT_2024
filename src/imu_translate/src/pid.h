#ifndef PID_H
#define PID_H


template<typename T>
void __LIMIT(T &a, const T &b)
{
    if (a > b) a = b;
    else if (a < -b) a = -b;
}
class PID
{
private:
    double kp,ki,kd,int_max,ctrl_max,int_sum;
    double last_error,last_delta_error;
    bool I_flag;
public:
    PID();
    double calc_output(double target,double actual);
    void setPID(double p,double i,double d);
    
};

#endif