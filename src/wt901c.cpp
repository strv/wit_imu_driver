#include "wit_imu_driver/wt901c.h"

namespace wit_imu_driver
{
Wt901c::Wt901c(const double co_gravity)
    : WitImu(co_gravity)
    , co_acc_(co_gravity_ * -16.0 / 32768.0)
    , co_avel_(M_PI * 2000.0 / (32768.0 * 180.0))
{
}

void Wt901c::pushBytes( const std::vector<uint8_t>& bytes,
                        const size_t size,
                        const ros::Time& stamp)
{
    buf_.insert(buf_.cend(), bytes.cbegin(), bytes.cbegin() + size);
    if (buf_.size() < 11)
    {
        // Not enough data length
        return;
    }
    while(buf_.size() >= 11)
    {
        if (buf_[0] != 0x55)
        {
            buf_.erase(buf_.cbegin());
            continue;
        }
        uint8_t cs = std::accumulate(buf_.begin(), buf_.begin() + 10, 0);
        if (cs != buf_[10])
        {
            std::printf("[Wt901c] Checksum is not match. Rx : %02X, Calc : %02X\n", buf_[10], cs);
            buf_.erase(buf_.cbegin());
            continue;
        }
        switch (buf_[1])
        {
            case 0x51:
            work_imu_ = sensor_msgs::Imu();
            work_imu_.header.stamp = stamp;
            work_imu_.linear_acceleration.x = co_acc_ * bytes2int(buf_[3], buf_[2]);
            work_imu_.linear_acceleration.y = co_acc_ * bytes2int(buf_[5], buf_[4]);
            work_imu_.linear_acceleration.z = co_acc_ * bytes2int(buf_[7], buf_[6]);
            break;

            case 0x52:
            work_imu_.angular_velocity.x = co_avel_ * bytes2int(buf_[3], buf_[2]);
            work_imu_.angular_velocity.y = co_avel_ * bytes2int(buf_[5], buf_[4]);
            work_imu_.angular_velocity.z = co_avel_ * bytes2int(buf_[7], buf_[6]);
            break;

            case 0x53:
            {
                double r,p,y;
                r = bytes2int(buf_[3], buf_[2]) * M_PI / 32768.0;
                p = bytes2int(buf_[5], buf_[4]) * M_PI / 32768.0;
                y = bytes2int(buf_[7], buf_[6]) * M_PI / 32768.0;
                tf2::Quaternion q;
                q.setRPY(r, p, y);
                q.normalize();
                work_imu_.orientation.x = q.x();
                work_imu_.orientation.y = q.y();
                work_imu_.orientation.z = q.z();
                work_imu_.orientation.w = q.w();
            }
            imu_buf_.push(work_imu_);
            break;

            case 0x54:
            break;
        }
        buf_.erase(buf_.cbegin(), buf_.cbegin() + 11);
    }
    return;
}
}   // wit_imu_driver
