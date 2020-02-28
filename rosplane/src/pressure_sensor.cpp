#include "pressure_sensor.h"
// Ensure that this libarary is installed https://github.com/amaork/libi2c


pressure_sensor::pressure_sensor()
{
  /* Open i2c bus /dev/i2c-0 */
  if ((static_bus_ = i2c_open("/dev/i2c-0")) == -1)
  {
  	ROS_ERROR("Failed to connect to I2C static pressure sensor!");
  }
  else
  {
    memset(&static_device_, 0, sizeof(static_device_));
    static_device_.bus = static_bus_;
    static_device_.addr = 0x10;
    static_device_.iaddr_bytes = 0;
    static_device_.page_bytes = 16;
  }

  if ((differential_bus_ = i2c_open("/dev/i2c-0")) == -1)
  {
  	ROS_ERROR("Failed to connect to I2C differential pressure sensor!");
  }
  else
  {
    memset(&static_device_, 0, sizeof(static_device_));
    static_device_.bus = static_bus_;
    static_device_.addr = 0x11;
    static_device_.iaddr_bytes = 0;
    static_device_.page_bytes = 16;
  }

  baro_pub_ = nh_.advertise<rosflight_msgs::Barometer>("baro",1);
  speed_pub_ = nh_.advertise<rosflight_msgs::Airspeed>("airspeed",1);

  while (ros::ok())
  {
    nh_.param("pressure_pub_rate",rate_,(float)100.0);
    ros::Timer timer = nh_.createTimer(ros::Duration(1.0/rate_), &pressure_sensor::publish_data, this);
    read_i2c();
  }

  i2c_close(static_bus_);
  i2c_close(differential_bus_);
}

void pressure_sensor::read_i2c()
{
  int temp1 = 0;
  int temp2 = 0;
  char buffer[4];
  ssize_t size = sizeof(buffer);
  memset(buffer, 0, sizeof(buffer));

  if ((i2c_ioctl_read(&static_device_, 0x0, buffer, size)) != 2) /* From i2c 0x0 address read 256 bytes data to buffer */
  {
    ROS_ERROR("Failed to read data from static i2c bus.");
  }
  else
  {
    float pres_count = (float)(((buffer[0] & 0b00111111)<<8)+buffer[1]);
    float temp_count = (float)((buffer[2]<<3)+((buffer[3] & 0b11100000)>>5));
    static_pressure_ = (pres_count-counts_max_)/(counts_max_-counts_min_)*(static_pres_max_-static_pres_min_)+static_pres_min_; //TODO Add conversion and check bit length above.
    static_temperature_ = temp_count*200.0/2048.0-50.0;
  }

  if ((i2c_ioctl_read(&differential_device_, 0x0, buffer, size)) != 2) /* From i2c 0x0 address read 256 bytes data to buffer */
  {
    ROS_ERROR("Failed to read data from differential i2c bus.");
  }
  else
  {
    float pres_count = (float)(((buffer[0] & 0b00111111)<<8)+buffer[1]);
    float temp_count = (float)((buffer[2]<<3)+((buffer[3] & 0b11100000)>>5));
    differential_pressure_ = (pres_count-counts_max_)/(counts_max_-counts_min_)*(differential_pres_max_-differential_pres_min_)+differential_pres_min_; //TODO Add conversion and check bit length above.
    differential_temperature_ = temp_count*200.0/2048.0-50.0;
  }
}

void pressure_sensor::publish_data(const ros::TimerEvent& e)
{
  rosflight_msgs::Barometer baro_msg;
  baro_msg.header.stamp = ros::Time::now();
  baro_msg.pressure = static_pressure_;
  baro_msg.temperature = static_temperature_;
  baro_pub_.publish(baro_msg);

  rosflight_msgs::Airspeed speed_msg;
  speed_msg.header.stamp = ros::Time::now();
  speed_msg.differential_pressure = differential_pressure_;
  speed_msg.temperature = differential_temperature_;
  speed_pub_.publish(speed_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pressure_sensor");
  pressure_sensor sensor = pressure_sensor();

  return 0;
}
