#if !defined(_MY_REMOTE_SHOOT_H__)
#define _MY_REMOTE_SHOOT_H__



const double PI = 3.1415926535;
const double default_k_1 = 0.008;  //! 原文给了一个k_1=0.1。似乎与实际情况不符，这里给出一个更小的k_1值


double getCompensationRad_xResistance(double target_x,double target_y,double velocity_0,bool log_flag=false,int N_times=10,double g=9.7803,double k_1=default_k_1);
double getCompensationRad_xyResistance1_shootup(double target_x,double target_y,double velocity_0,bool log_flag=false,int N_times=10,double g=9.7803,double k_1=default_k_1);
double getCompensationRad_xyResistance2_shootup(double target_x,double target_y,double velocity_0,bool log_flag=false,int N_times=10,double g=9.7803,double k_1=default_k_1);
double getCompensationRad_xyResistance2_shootdown(double target_x,double target_y,double velocity_0,bool log_flag=false,int N_times=10,double g=9.7803,double k_1=default_k_1);
double getCompensationRad_xyResistance2(double target_x,double target_y,double velocity_0,bool log_flag=false,int N_times=10,double g=9.7803,double k_1=default_k_1);


#endif // _MY_REMOTE_SHOOT_H__
