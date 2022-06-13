
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <memory>
#include <vector>
#include <string>
#include <functional>
#include <chrono>

#include "ichthus_driver/SocketCAN.h"
#include "ichthus_driver/kia_reader.hpp"

namespace ichthus
{

IchthusCANKIAReader::IchthusCANKIAReader(const rclcpp::NodeOptions & options)
: rclcpp::Node("IchthusCANReader", options)
{
  RCLCPP_INFO(this->get_logger(), "===Start CAN Reader===");
  init_Param();
  write_buffer = (double *)malloc(sizeof(double) * PAYLOADSIZE);
  memset(write_buffer, 0, sizeof(float) * PAYLOADSIZE);

  adapter = new SocketCAN();
  adapter->open(&*can_interface.begin());

  load_dbc(ecu_ids, dbc_file);

  can_pub = this->create_publisher<ichthus_msgs::msg::Can>("odom_raw", 1);

  //cb_handle = this->add_on_set_parameters_callback(
  //  std::bind(&IchthusCANKIAReader::param_CB, this, std::placeholders::_1));

  rx_thread = std::thread(&IchthusCANKIAReader::rxThread, this);
}

IchthusCANKIAReader::~IchthusCANKIAReader()
{
  delete adapter;
  if (rx_thread.joinable())
  {
    RCLCPP_INFO(this->get_logger(), "Thread is joinable");
    std::terminate();
    //rx_thread.join();
  }
  RCLCPP_INFO(this->get_logger(), "Class Destructed");
}

void IchthusCANKIAReader::init_Param()
{
  this->declare_parameter("dbc_file", (std::string)"/home/jjong/colcon_ws/src/ichthus_driver/src/hyundai_kia_generic.dbc");
  this->declare_parameter("can_interface", (std::string)"vcan1");

  dbc_file = this->get_parameter("dbc_file").as_string();
  can_interface = this->get_parameter("can_interface").as_string();
  ecu_ids.push_back(688);//2b0 - SAS Angle & Speed
  ecu_ids.push_back(902);//386 - 4 Wheel Speed
  ecu_ids.push_back(882);//372 - Vehicle Gear
  ecu_ids.push_back(544);//220 - LonLat Accel & Yaw Rate

  sas_ang_idx = 0;
  sas_spd_idx = 1;
  whl4_spd_len = 4;
  lat_acc_idx = 0;
  lon_acc_idx = 3;
  yaw_rate_idx = 9;
  gear_idx = 0;

  current_kmph = 0;

  can_msg.can_data.clear();
}

void IchthusCANKIAReader::load_dbc()
{

  {
    std::ifstream idbc("hyundai_kia_generic.dbc");
    net = dbcppp::INetwork::LoadDBCFromIs(idbc);
  }

  for (const dbcppp::IMessage& msg : net->Messages())
  {
    messages.insert(std::make_pair(msg.Id(), &msg));
  }
}

void IchthusCANKIAReader::load_dbc(std::vector<uint64_t> & ids, const std::string & dbc)
{
  {
    std::ifstream idbc(dbc);
    net = dbcppp::INetwork::LoadDBCFromIs(idbc);
    RCLCPP_INFO(this->get_logger(), "Load DBC File");
  }

  for (const dbcppp::IMessage& msg : net->Messages())
  {
    auto iter = find(ids.begin(), ids.end(), msg.Id());

    if (iter != ids.end())
    {
      messages.insert(std::make_pair(msg.Id(), &msg));
    }
  }
  RCLCPP_INFO(this->get_logger(), "Complete DBC file");
}

void IchthusCANKIAReader::rx_kia_Handler(can_frame_t* frame)
{
  auto iter = messages.find(frame->can_id);
  const dbcppp::IMessage * msg;
  const dbcppp::ISignal * mux_sig;
  std_msgs::msg::Float64MultiArray msgs;
  std::vector<double> can_msg;

  if (iter != messages.end())
  {
    msg = iter->second;

    for (const dbcppp::ISignal & sig : msg->Signals())
    {
      mux_sig = msg->MuxSignal();

      if (sig.MultiplexerIndicator() != dbcppp::ISignal::EMultiplexer::MuxValue ||
           (mux_sig && mux_sig->Decode(frame->data) == sig.MultiplexerSwitchValue()))
      {
        /* Convert Data to ROS Message */
        msgs.data.push_back(sig.RawToPhys(sig.Decode(frame->data)));
      }
    }
    switch (iter->first)
    {
      case 688:
        ang_handler(msgs);
        break;
      case 902:
        spd_handler(msgs);
        break;
      case 882:
        gear_handler(msgs);
        break;
      case 544:
        acc_handler(msgs);
        break;
    }
  }
}

void IchthusCANKIAReader::ang_handler(std_msgs::msg::Float64MultiArray msgs)
{
  memcpy(write_buffer, &msgs.data[sas_ang_idx], sizeof(double));
  memcpy(write_buffer+1, &msgs.data[sas_spd_idx], sizeof(double));
  //RCLCPP_INFO(this->get_logger(), "ang_handler.");
}

void IchthusCANKIAReader::spd_handler(std_msgs::msg::Float64MultiArray msgs)
{
  int idx = 0;
  for (auto whl_spd = msgs.data.begin(); idx < whl4_spd_len; idx++, whl_spd++)
  {
    current_kmph += *whl_spd;
  }
  current_kmph /= whl4_spd_len;
  memcpy(write_buffer+2, &current_kmph, sizeof(double));
  //RCLCPP_INFO(this->get_logger(), "spd_handler.");
}

void IchthusCANKIAReader::acc_handler(std_msgs::msg::Float64MultiArray msgs)
{
  std_msgs::msg::Header head;
  auto time_now = this->now();
  head.stamp = time_now;
  times.push_back(head);
  memcpy(write_buffer+3, &msgs.data[lat_acc_idx], sizeof(double));
  memcpy(write_buffer+4, &msgs.data[lon_acc_idx], sizeof(double));
  memcpy(write_buffer+5, &msgs.data[yaw_rate_idx], sizeof(double));
  publishOdom();
  
  //RCLCPP_INFO(this->get_logger(), "acc_handler.");
}

void IchthusCANKIAReader::gear_handler(std_msgs::msg::Float64MultiArray msgs)
{
  memcpy(write_buffer+6, &msgs.data[gear_idx], sizeof(double));
  //RCLCPP_INFO(this->get_logger(), "gear_handler.");
}

void IchthusCANKIAReader::publishOdom()
{
  can_msg.head.stamp = times.front().stamp;
  for(int i=0; i<PAYLOADSIZE; ++i){
    can_msg.can_data.push_back(write_buffer[i]);
  }
  can_msg.can_names.push_back("STR_ANG");
  can_msg.can_names.push_back("STR_VEL");
  can_msg.can_names.push_back("CUR_VEL");
  can_msg.can_names.push_back("LAT_ACC");
  can_msg.can_names.push_back("LON_ACC");
  can_msg.can_names.push_back("YAW_RATE");
  can_msg.can_names.push_back("GEAR");
  can_pub->publish(can_msg);

  can_msg.can_data.clear();
  can_msg.can_names.clear();
  times.clear();
  RCLCPP_INFO(this->get_logger(), "STR_ANG %f", can_msg.can_data[0]);
  RCLCPP_INFO(this->get_logger(), "STR_VEL %f", can_msg.can_data[1]);
  RCLCPP_INFO(this->get_logger(), "CUR_VEL %f", can_msg.can_data[2]);
  RCLCPP_INFO(this->get_logger(), "LAT_ACC %f", can_msg.can_data[3]);
  RCLCPP_INFO(this->get_logger(), "LON_ACC %f", can_msg.can_data[4]);
  RCLCPP_INFO(this->get_logger(), "YAW_RATE %f", can_msg.can_data[5]);
  RCLCPP_INFO(this->get_logger(), "GEAR %f", can_msg.can_data[6]);
}

bool IchthusCANKIAReader::rx()
{
  can_frame_t frame;
  if (adapter->receive(frame))
  {
    rx_kia_Handler(&frame);
    return true;
  }
  else
  {
    return false;
  }
}

void IchthusCANKIAReader::rxThread()
{
  do {
    rx();
  } while (rclcpp::ok());
}

rcl_interfaces::msg::SetParametersResult
IchthusCANKIAReader::param_CB(const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  for (const auto &param : params)
  {
    if (param.get_name() == "can_interface")
    {
      can_interface = param.as_string();
      RCLCPP_INFO(this->get_logger(), "change CAN Interface");
      adapter->open(&*can_interface.begin());
    }
    else if (param.get_name() == "dbc_file")
    {
      dbc_file = param.as_string();
      RCLCPP_INFO(this->get_logger(), "change DBC File");
      load_dbc(ecu_ids, dbc_file);
    }
  }
  return result;
}

} //namespace end
