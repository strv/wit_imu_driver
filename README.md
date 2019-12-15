kinetic : [![CircleCI](https://circleci.com/gh/strv/wit_imu_driver.svg?style=svg)](https://circleci.com/gh/strv/wit_imu_driver)

# wit_imu_driver

Unofficial ROS driver for WIT Motion's IMU.

## Supported device

- WT901 series

WIP to add other device.

# parameters

- ```gravity```
    - Gravity at your site.
- ```frame_id```
    - frame_id of output topics.
- ```device```
    - Device name to communicate. Default : ```/dev/ttyUSB0```
- ```baud```
    - Communication baudrate. Default : ```9600```

# topics

## publish

- ```data_raw```
- ```temperature```
- ```mag```

## subscribe

None

# TODO

- Add configuration featurues
- Add other device
