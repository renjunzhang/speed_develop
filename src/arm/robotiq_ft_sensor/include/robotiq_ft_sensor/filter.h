#ifndef FILTER_CLASS_H
#define FILTER_CLASS_H

#include <iostream>

#include <Eigen/Dense>

#include <ros/ros.h>

using namespace Eigen;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<int, 6, 1> Vector6i;

class Filter
{
public:
    Vector6d LowPassFilter(const Vector6d &data);
    Vector6d DynLowPassFilter(const Vector6d &data);
public:
    Filter(){};
    ~Filter(){};
    void init(const ros::NodeHandle &nh);
// LowPassFilter Params:
private:
    Vector6d             Filter_K;
    Vector6d             Filter_K_d;
    Vector6d             LocalThres;
    Vector6d             GlobalThres;
    Vector6d             AddSum;
    Vector6d             AddNum;
    Vector6i             OldFlag;
    Vector6i             NewFlag;

private:
    ros::NodeHandle             nh;
    Vector6d             OldData;
    Vector6d             NewData;
};

#endif