/*
 * Copyright(c) 2019, strv
 * All rights reserved.
 */

#ifndef WIT_IMU_DRIVER_WT901C_H
#define WIT_IMU_DRIVER_WT901C_H

#include <wit_imu_driver/wit_imu.h>

#include <queue>
#include <vector>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>

namespace wit_imu_driver
{
class Wt901c : public WitImu
{
public:
    explicit Wt901c(const double co_gravity = 9.8);
    void pushBytes(const std::vector<uint8_t>& bytes,
                    const size_t size,
                    const ros::Time& stamp);
    virtual std::vector<uint8_t> genYawClr() const;
    virtual std::vector<uint8_t> genHightClr() const;
    virtual std::vector<uint8_t> genAccCal() const;
    virtual std::vector<uint8_t> genMagCal() const;
    virtual std::vector<uint8_t> genExitCal() const;
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
}   // namespace wit_imu_driver

#endif  // WIT_IMU_DRIVER_WT901C_H
