/*
 * Copyright(c) 2019, strv
 * All rights reserved.
 */
#ifndef WIT_IMU_DRIVER_WIT_IMU_H
#define WIT_IMU_DRIVER_WIT_IMU_H

#include <vector>
#include <queue>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/MagneticField.h>

namespace wit_imu_driver
{
enum PRODUCT
{
    WT901C
};

class WitImu
{
public:
    WitImu(const double co_gravity)
    : buf_(1024)
    , co_gravity_(co_gravity)
    {
    }

    virtual void pushBytes( const std::vector<uint8_t>& bytes,
                            const size_t size,
                            const ros::Time& stamp){};
    bool popImuData(sensor_msgs::Imu* const p_msg)
    {
        if (imu_buf_.empty())
        {
            return false;
        }
        *p_msg = imu_buf_.front();
        imu_buf_.pop();
        return true;
    };

    size_t sizeImuData()
    {
        return imu_buf_.size();
    };

protected:
    double co_gravity_;
    std::vector<uint8_t> buf_;
    std::queue<sensor_msgs::Imu> imu_buf_;

    static int bytes2int(const uint8_t h, const uint8_t l)
    {
        return static_cast<int16_t>(
                ((static_cast<uint16_t>(h) << 8) & 0xFF00)
                | (static_cast<uint16_t>(l) & 0x00FF));
    }
};
}   // wit_imu_driver

#endif // WIT_IMU_DRIVER_WIT_IMU_H