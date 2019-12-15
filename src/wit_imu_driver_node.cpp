/*
 * Copyright(c) 2019, strv
 * All rights reserved.
 */

#include <mutex>
#include <thread>
#include <vector>
#include <queue>
#include <numeric>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <tf2/LinearMath/Quaternion.h>

#include <boost/asio.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/bind.hpp>
#include <linux/serial.h>
#include <boost/shared_ptr.hpp>

namespace wit_imu_driver
{
namespace ba = boost::asio;

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

class Wt901c : public WitImu
{
public:
    Wt901c(const double co_gravity)
    : WitImu(co_gravity)
    , co_acc_(co_gravity_ * -16.0 / 32768.0)
    , co_avel_(M_PI * 2000.0 / (32768.0 * 180.0))
    {
    }

    void pushBytes( const std::vector<uint8_t>& bytes,
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
private:
    std::queue<sensor_msgs::Temperature> temp_buf_;
    sensor_msgs::Imu work_imu_;
    const double co_acc_;
    const double co_avel_;
};

class WitImuDriver
{
public:
    WitImuDriver()
    : nh_()
    , pnh_("~")
    , port_io_()
    , port_(port_io_)
    , wdg_timeout_(0.5)
    , rx_buf_(1024)
    {
        pub_imu_ = nh_.advertise<sensor_msgs::Imu>("data_raw", 10);
        pub_temp_ = nh_.advertise<sensor_msgs::Temperature>("temperature", 10);
        pnh_.param<double>("gravity", co_gravity_, 9.797673);
        pnh_.param<std::string>("frame_id", frame_id_, "imu_link");
    }

    bool open()
    {
        std::string dev;
        int baud;
        pnh_.param<std::string>("device", dev, "/dev/ttyUSB0");
        pnh_.param<int>("baud", baud, 9600);

        boost::system::error_code ec;
        port_.open(dev, ec);
        if (ec.value() != 0)
        {
            ROS_ERROR("Failed to open %s. Error code : %d",
                    dev.c_str(),
                    ec.value());
            return false;
        }
        port_.set_option(ba::serial_port_base::baud_rate(baud), ec);
        if (ec.value() != 0)
        {
            ROS_ERROR("Failed to set baudrate. Error code : %d",
                    ec.value());
            return false;
        }
        port_.set_option(ba::serial_port_base::character_size(8), ec);
        if (ec.value() != 0)
        {
            ROS_ERROR("Failed to set options. Error code : %d",
                    ec.value());
            return false;
        }
        port_.set_option(ba::serial_port_base::flow_control(
                            ba::serial_port_base::flow_control::none), ec);
        if (ec.value() != 0)
        {
            ROS_ERROR("Failed to set options. Error code : %d",
                    ec.value());
            return false;
        }
        port_.set_option(ba::serial_port_base::parity(
                            ba::serial_port_base::parity::none), ec);
        if (ec.value() != 0)
        {
            ROS_ERROR("Failed to set options. Error code : %d",
                    ec.value());
            return false;
        }
        port_.set_option(ba::serial_port_base::stop_bits(
                            ba::serial_port_base::stop_bits::one), ec);
        if (ec.value() != 0)
        {
            ROS_ERROR("Failed to set options. Error code : %d",
                    ec.value());
            return false;
        }
        // FTDI USB-serial device has 16 msec latency in default.
        // It makes large latency and jitter for high rate measurement.
        const int fd = port_.native_handle();
        serial_struct port_info;
        ioctl(fd, TIOCGSERIAL, &port_info);
        port_info.flags |= ASYNC_LOW_LATENCY;
        ioctl(fd, TIOCSSERIAL, &port_info);

        return true;
    }

    bool spin()
    {
        ptr_imu_ = boost::make_shared<Wt901c>(Wt901c(co_gravity_));
        startRead();
        auto io_run = [this]()
        {
            boost::system::error_code ec;
            port_io_.run(ec);
        };
        io_thread_ = std::thread(io_run);

        wdg_ = pnh_.createTimer(wdg_timeout_, &WitImuDriver::cbWdg, this, true, false);

        ros::spin();
        close();
        return true;
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher pub_imu_;
    ros::Publisher pub_temp_;
    ros::Timer wdg_;
    ros::Duration wdg_timeout_;
    std::string frame_id_;

    ba::io_service port_io_;
    ba::serial_port port_;
    std::thread io_thread_;
    std::mutex mtx_;

    double co_gravity_;
    std::vector<uint8_t> rx_buf_;

    boost::shared_ptr<WitImu> ptr_imu_;

    void close()
    {
        if (port_.is_open())
        {
            port_.cancel();
            port_.close();
        }
        if (!port_io_.stopped())
        {
            port_io_.stop();
        }
        if (io_thread_.joinable())
        {
            io_thread_.join();
        }
    }

    void startRead()
    {
        ba::async_read(port_,
                    ba::buffer(rx_buf_),
                    ba::transfer_at_least(1),
                    boost::bind(&WitImuDriver::cbPort,
                                this,
                                ba::placeholders::error,
                                ba::placeholders::bytes_transferred));
    }

    void cbPort(const boost::system::error_code& ec,
                std::size_t size)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!ec)
        {
            ptr_imu_->pushBytes(rx_buf_, size, ros::Time::now());
            while (ptr_imu_->sizeImuData() != 0)
            {
                sensor_msgs::Imu msg;
                ptr_imu_->popImuData(&msg);
                msg.header.frame_id = frame_id_;
                pub_imu_.publish(msg);
            }
            startRead();
            resetWdg();
        }
        else if (ec == boost::system::errc::operation_canceled)
        {
            // Enter to this state when stop a connection
        }
        else
        {
            // Unknown state
            ROS_ERROR("[wit_imu_driver] seiral error : %s", ec.message().c_str());
            ros::shutdown();
        }
    }

    void cbWdg(const ros::TimerEvent& event)
    {
        if (port_.is_open())
        {
            ROS_ERROR("Timeouted. No data received from IMU.");
            ros::shutdown();
        }
    }

    void resetWdg()
    {
        wdg_.stop();
        wdg_.setPeriod(wdg_timeout_);
        wdg_.start();
    }
};
}   // wit_imu_driver

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wit_imu_driver");
    ROS_INFO("Start wit_imu_driver");

    wit_imu_driver::WitImuDriver imu;
    if (!imu.open())
    {
        ROS_ERROR("[wit_imu_driver] Failed to open");
        return 1;
    }

    if (!imu.spin())
    {
        ROS_ERROR("[wit_imu_driver] Exit by error");
        return 1;
    }

    return 0;
}
