#ifndef PRESSURE_SENSOR_H
#define PRESSURE_SENSOR_H

#include <linux/i2c-dev.h>
#include <ros/ros.h>
#include <rosflight_msgs/Barometer.h>
#include <rosflight_msgs/Airspeed.h>
#include <i2c.h>

class pressure_sensor
{
public:
  pressure_sensor();

private:
  ros::NodeHandle nh_;
  ros::Publisher baro_pub_;
  ros::Publisher speed_pub_;
  void read_i2c();
  void publish_data(const ros::TimerEvent& e);
  int static_bus_;
  int differential_bus_;
  I2CDevice static_device_;
  I2CDevice differential_device_;
  int static_pressure_;
  int differential_pressure_;
  int static_temperature_;
  int differential_temperature_;
  float rate_;
  float counts_max_ = 14745.0;
  float counts_min_ = 1638.0;
  float static_pres_max_ = 120000;
  float static_pres_min_ = 70000;
  float differential_pres_max_ = 2000;
  float differential_pres_min_ = 0;
};

#endif
