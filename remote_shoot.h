#if !defined(_MY_REMOTE_SHOOT_H__)
#define _MY_REMOTE_SHOOT_H__

const double PI = 3.1415926535;

double getCompensationRad_xResistance(double target_x,double target_y,double velocity_0,bool log_flag=false,int N_times=10,double g=9.7803,double k_1=0.1);


#endif // _MY_REMOTE_SHOOT_H__
