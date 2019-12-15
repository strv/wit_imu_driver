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
    std::queue<sensor_msgs::Temperature> temp_buf_;
    sensor_msgs::Imu work_imu_;
    const double co_acc_;
    const double co_avel_;
};
}   // wit_imu_driver

#endif // WIT_IMU_DRIVER_WT901C_H