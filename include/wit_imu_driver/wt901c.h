#ifndef WIT_IMU_DRIVER_WT901C_H
#define WIT_IMU_DRIVER_WT901C_H

#include "wit_imu.h"

#include <vector>
#include <queue>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/MagneticField.h>
#include <tf2/LinearMath/Quaternion.h>

namespace wit_imu_driver
{
class Wt901c : public WitImu
{
public:
    Wt901c(const double co_gravity);
    void pushBytes( const std::vector<uint8_t>& bytes,
                    const size_t size,
                    const ros::Time& stamp);
private:
    sensor_msgs::Imu work_imu_;
    sensor_msgs::Temperature work_temp_;
    sensor_msgs::MagneticField work_mag_;
    const double co_acc_;
    const double co_avel_;
    const double co_temp_;
    const double co_mag_;
    const double co_pose_;
};
}   // wit_imu_driver

#endif // WIT_IMU_DRIVER_WT901C_H