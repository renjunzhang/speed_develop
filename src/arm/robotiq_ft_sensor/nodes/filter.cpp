#include "robotiq_ft_sensor/filter.h"

void Filter::init(const ros::NodeHandle &nh)
{
    std::vector<double> Filter_K_;
    std::vector<double> Filter_K_d_;
    std::vector<double> LocalThres_;
    std::vector<double> GlobalThres_;
    std::vector<double> AddNum_;
    if (!nh.getParam("Filter_K", Filter_K_)) { ROS_ERROR("Couldn't retrieve the Filter K.");}
    if (!nh.getParam("Filter_K_d", Filter_K_d_)) { ROS_ERROR("Couldn't retrieve the Filter K_d.");}
    if (!nh.getParam("LocalThres", LocalThres_)) { ROS_ERROR("Couldn't retrieve the Local Threshold.");}
    if (!nh.getParam("GlobalThres", GlobalThres_)) { ROS_ERROR("Couldn't retrieve the Global Threshold.");}
    if (!nh.getParam("AddNum", AddNum_)) { ROS_ERROR("Couldn't retrieve the Add Number.");}
    Filter_K = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(Filter_K_.data(), Filter_K_.size());
    Filter_K_d = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(Filter_K_d_.data(), Filter_K_d_.size());
    LocalThres = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(LocalThres_.data(), LocalThres_.size());
    GlobalThres = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(GlobalThres_.data(), GlobalThres_.size());
    AddNum = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(AddNum_.data(), AddNum_.size());

    OldData.setZero();
    OldFlag.setZero();
    NewFlag.setZero();
    AddSum.setZero();
}

Vector6d Filter::LowPassFilter(const Vector6d &data)
{
    // *低通滤波
    for (size_t i = 0; i < 6; i++)
    {
        NewData(i) = (1 - Filter_K(i))*OldData(i) + Filter_K(i)*NewData(i);
    }
    OldData = NewData;
    return NewData;
}

Vector6d Filter::DynLowPassFilter(const Vector6d &data)
{
    // *渐近增量低通滤波，详见 https://blog.csdn.net/WilliamCode/article/details/78699458
    for (size_t i = 0; i < 6; i++)
    {
        NewData(i) = data(i);
        // 新数据和原数据对比
        if((NewData(i) - OldData(i)) > 0)
        {
            NewFlag(i) = 1;
        }
        else
        {
            NewFlag(i) = 0;
        }
        // 如果数据连续增大（减小）
        if (NewFlag(i) == OldFlag(i))
        {
            // 如果数据和原数据相差较大，AddSum累加
            if (fabs(NewData(i) - OldData(i)) > LocalThres(i))
            {
                AddSum(i) += AddNum(i);
            }
            // 大于阈值，则更新滤波参数
            if (AddSum(i) > GlobalThres(i))
            {
                Filter_K(i) += Filter_K_d(i);
            }
        }
        else
        {
            AddSum(i) = 0;
            // TODO:直接设为K_d是否合适？
            Filter_K(i) = Filter_K_d(i);
            OldFlag(i) = NewFlag(i);
        }

        if (Filter_K(i) > 0.95)
        {
            Filter_K(i) = 0.95;
        }
        NewData(i) = ( 1 - Filter_K(i)) * OldData(i) + Filter_K(i) * NewData(i);
        OldData(i) = NewData(i);
    }
}