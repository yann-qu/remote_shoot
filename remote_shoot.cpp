#include <iostream>
#include <cmath>
#include <glog/logging.h>
#include "remote_shoot.h"


/**
 * @Date: 2021-01-16 18:26:49
 * @description: 通过x方向阻力模型计算射击补偿角
 * @param double target_x: x方向的距离
 * @param double target_y: y方向的距离
 * @param double velocity_0: 子弹射速。可用上一发子弹射速来近似
 * @param bool log_flag=false: 日志开关
 * @param int N_times: 最大迭代次数
 * @param double g: 重力加速度
 * @param double k_1: f=k_0*v^2,k_1=k_0/m
 * @return double compensation_rad:射击补偿角的弧度值
 */
double getCompensationRad_xResistance(double target_x,double target_y,double velocity_0,bool log_flag,int N_times,double g,double k_1){

    // 初始化glog
    if(log_flag){
        FLAGS_logtostderr=true;
        FLAGS_colorlogtostderr=true;
    }

    // 临时点
    double temp_x,temp_y;
    // 实际命中点
    double real_x,real_y;
    // 高度偏差
    double delta_H=0;
    // 运动时间
    double time;
    // 初始角弧度值
    double theta_0=atan(target_y/target_x);
    // 补偿角弧度值
    double compensation_rad=0;

    temp_x=target_x;
    temp_y=target_y;

    for(int i=0; i<N_times; i++){
        // 获取子弹x方向运动时间
        time=(exp(k_1*target_x)-1)/(k_1*(velocity_0*cos(theta_0+compensation_rad)));

        // 计算实际命中点
        real_y=(velocity_0*sin(theta_0+compensation_rad))*time-g/2*time*time;

        delta_H=target_y-real_y;

        // 更新角度和目标点
        temp_y+=delta_H;
        compensation_rad=atan(temp_y/target_x)-theta_0;
        LOG_IF(INFO,log_flag==true)<<"i="<<i<<" temp_y="<<temp_y<<" delta_H="<<delta_H<<" compensation_rad="<<compensation_rad<<"="<<compensation_rad*180/PI<<"degree"<<" pitch="<<compensation_rad+theta_0<<"="<<(compensation_rad+theta_0)*180/PI<<"degree";
    }


    return compensation_rad;
}
