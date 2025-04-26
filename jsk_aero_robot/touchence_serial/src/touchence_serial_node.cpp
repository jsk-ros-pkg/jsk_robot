// Copyright (c) 2017 Horisu
// https://github.com/Horisu/touchence_serial

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_service_options.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <boost/asio.hpp>
#include <iostream>

#include <mutex>
#include <std_srvs/Trigger.h>

const double matrix28_[9] = {9.910925439, 1.231185466, 0.970859841, -1.973447799, 14.66994936, -0.921719128, 0.630990042, -0.77121074, 55.61331789};
const double matrix33_[9] = {11.38555711, 1.075823951, 0.341612057, -1.334627966, 13.00710247, -2.036977959, 0.909452516, -2.554647371, 51.72511683};
const double matrix8_[9] = {11.15747603, 1.216047695, 2.318814325, -1.728443747, 12.10689728, 0.095171634, 0.188682325, -1.611091136, 51.28225561};
const double* matrixes_[3] = {matrix28_, matrix8_, matrix33_};

boost::asio::io_service io_;
boost::asio::serial_port port_(io_, "/dev/touchsensor");

std::mutex io_mutex_;
double calib_offset_[3][3];
double calib_temp_offset_[3];

bool calibrate(std_srvs::Trigger::Request &_req,
               std_srvs::Trigger::Response &_res) {
  std::cout << "calibration start" << std::endl;

  double calib_tmp[3][3], calib_temp_tmp[3];
  for (int i = 0; i < 3; ++i) {
    calib_tmp[i][0] = 0;
    calib_tmp[i][1] = 0;
    calib_tmp[i][2] = 0;
    calib_temp_tmp[i] = 0;
  }

  io_mutex_.lock();
  char* e;
  std::size_t length;
  int calib_count = 0;
  long long_num[3], long_temp;
  boost::asio::streambuf rbuf;
  for (int i = 0; i < 50; ++i) {
    std::cout << "call" << std::endl;
    port_.write_some(boost::asio::buffer("020201"));
    length = boost::asio::read_until(port_,rbuf,'\n');
    std::cout << boost::asio::buffer_cast<const char*>(rbuf.data()) << std::endl;

    std::cout << length << std::endl;
    std::string tmp = boost::asio::buffer_cast<const char*>(rbuf.data());
    std::cout << tmp << std::endl;
    if (length == 68) {
      for (int i = 0; i < 3; ++i) {
        long_num[0] = std::strtol(tmp.substr(2 + i*16,4).c_str(), &e, 16);
        long_num[1] = std::strtol(tmp.substr(6 + i*16,4).c_str(), &e, 16);
        long_num[2] = std::strtol(tmp.substr(10 + i*16,4).c_str(), &e, 16);
        long_temp = std::strtol(tmp.substr(14 + i*16,4).c_str(), &e, 16);

        calib_tmp[i][0] += matrixes_[i][0]*long_num[0] + matrixes_[i][1]*long_num[1] + matrixes_[i][2]*long_num[2];
        calib_tmp[i][1] += matrixes_[i][3]*long_num[0] + matrixes_[i][4]*long_num[1] + matrixes_[i][5]*long_num[2];
        calib_tmp[i][2] += matrixes_[i][6]*long_num[0] + matrixes_[i][7]*long_num[1] + matrixes_[i][8]*long_num[2];
        calib_temp_tmp[i] += long_temp;
      }
      ++calib_count;
    }
    rbuf.consume(length);
    usleep(100 * 1000);
  }

  if (calib_count == 0) {
    ROS_ERROR("can't read force");
    _res.success = false;
    return true;
  }

  for (int i = 0; i < 3; ++i) {
    calib_offset_[i][0] = calib_tmp[i][0]/calib_count;
    calib_offset_[i][1] = calib_tmp[i][1]/calib_count;
    calib_offset_[i][2] = calib_tmp[i][2]/calib_count;
    calib_temp_offset_[i] = calib_temp_tmp[i]/calib_count;
  }
  io_mutex_.unlock();
  std::cout << "calibration finished" << std::endl;

  _res.success = true;
  return true;
}

int main(int argc,char** argv) {

  ros::init(argc,argv,"touch_sensor");
  ros::NodeHandle nh;

  ros::Publisher force1_pub = nh.advertise<geometry_msgs::Vector3>("touchence/force01", 100);
  ros::Publisher force2_pub = nh.advertise<geometry_msgs::Vector3>("touchence/force02", 100);
  ros::Publisher force3_pub = nh.advertise<geometry_msgs::Vector3>("touchence/force03", 100);
  ros::Publisher force_pubs[3] = {force1_pub, force2_pub, force3_pub};

  ros::Publisher temp1_pub = nh.advertise<std_msgs::Float32>("touchence/temp01", 100);
  ros::Publisher temp2_pub = nh.advertise<std_msgs::Float32>("touchence/temp02", 100);
  ros::Publisher temp3_pub = nh.advertise<std_msgs::Float32>("touchence/temp03", 100);
  ros::Publisher temp_pubs[3] = {temp1_pub, temp2_pub, temp3_pub};

  ros::CallbackQueue calib_queue;
  ros::AsyncSpinner calib_spinner(1, &calib_queue);
  ros::AdvertiseServiceOptions calib_ops =
    ros::AdvertiseServiceOptions::create<std_srvs::Trigger>(
        "/touchence/calibrate",
        boost::bind(&calibrate, _1, _2),
        ros::VoidPtr(),
        &calib_queue);
  ros::ServiceServer calib_server = nh.advertiseService(calib_ops);
  calib_spinner.start();

  port_.set_option(boost::asio::serial_port_base::baud_rate(230400));
  port_.set_option(boost::asio::serial_port_base::character_size(8));
  port_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
  port_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
  port_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));

  //port.write_some(boost::asio::buffer(send_bin));

  // calibrate sensors
  std::cout << "calibration ready" << std::endl;
  std::cout << "dont touch the force sensors" << std::endl;
  std_srvs::Trigger srv;
  calibrate(srv.request, srv.response);
  if (!srv.response.success)
    return -1;

  // main loop
  char* e;
  std::size_t length;
  long long_num[3], long_temp;
  boost::asio::streambuf rbuf;
  double temp_k[3] = {-12,-5,-9};
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    io_mutex_.lock();
    port_.write_some(boost::asio::buffer("020201"));
    length = boost::asio::read_until(port_,rbuf,'\n');
    std::cout << boost::asio::buffer_cast<const char*>(rbuf.data()) << std::endl;
    
    std::cout << length << std::endl;
    std::string tmp = boost::asio::buffer_cast<const char*>(rbuf.data());
    std::cout << tmp << std::endl;

    if (length == 68) {
      for (int i = 0; i < 3; ++i) {
        geometry_msgs::Vector3 tmp_msg;
        std_msgs::Float32 temp_msg;

        long_num[0] = std::strtol(tmp.substr(2 + i*16,4).c_str(),&e,16);
        long_num[1] = std::strtol(tmp.substr(6 + i*16,4).c_str(),&e,16);
        long_num[2] = std::strtol(tmp.substr(10 + i*16,4).c_str(),&e,16);
        long_temp = std::strtol(tmp.substr(14 + i*16,4).c_str(),&e,16);

        tmp_msg.x = matrixes_[i][0]*long_num[0] + matrixes_[i][1]*long_num[1] + matrixes_[i][2]*long_num[2] - calib_offset_[i][0];
        tmp_msg.y = matrixes_[i][3]*long_num[0] + matrixes_[i][4]*long_num[1] + matrixes_[i][5]*long_num[2] - calib_offset_[i][1];
        tmp_msg.z = matrixes_[i][6]*long_num[0] + matrixes_[i][7]*long_num[1] + matrixes_[i][8]*long_num[2] - calib_offset_[i][2] - (long_temp - calib_temp_offset_[i]) * temp_k[i];
        temp_msg.data = long_temp - calib_temp_offset_[i];

        // 10bit3.3V maybe
        tmp_msg.x /= 310;
        tmp_msg.y /= 310;
        tmp_msg.z /= 310;
        temp_msg.data /= 310;

        force_pubs[i].publish(tmp_msg);
        temp_pubs[i].publish(temp_msg);
      }
      std::cout << std::endl;
    }
    rbuf.consume(length);
    io_mutex_.unlock();
    loop_rate.sleep();
  }
  return 0;
}
