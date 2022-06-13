#ifndef ICHTHUS_CAN__KIA_READER_HPP_
#define ICHTHUS_CAN__KIA_READER_HPP_  

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/header.hpp>

#include <unistd.h>

#include <algorithm>
#include <memory>
#include <vector>
#include <string>
#include <unordered_map>
#include <fstream>
#include <stdlib.h>

#include "ichthus_msgs/msg/can.hpp"
#include "ichthus_driver/SocketCAN.h"
#include "ichthus_driver/SLCAN.h"
#include "dbcppp/CApi.h"
#include "dbcppp/Network.h"

#define PAYLOADSIZE 7 

namespace ichthus
{

class IchthusCANKIAReader : public rclcpp::Node
{
  private:
    rclcpp::Publisher<ichthus_msgs::msg::Can>::SharedPtr can_pub;

    OnSetParametersCallbackHandle::SharedPtr cb_handle;

    std::unique_ptr<dbcppp::INetwork> net;
    std::unordered_map<uint64_t, const dbcppp::IMessage *> messages;

    SocketCAN * adapter;

    std::string can_interface;
    std::string dbc_file;
    std::vector<uint64_t> ecu_ids;
    std::thread rx_thread;

    std::vector<std_msgs::msg::Header> times;
    ichthus_msgs::msg::Can can_msg;


    /*
    *  Write buffer payload
    *  buffer[0] = STEER ANGLE  (deg)
    *  buffer[1] = STEER VEL    (???)
    *  buffer[2] = CUR VEL      (km/h)
    *  buffer[3] = LAT ACC      (m/s^2)
    *  buffer[4] = LON ACC      (m/s^2)
    *  buffer[5] = YAW          (deg/s)
    *  buffer[6] = GEAR
    */
    double* write_buffer;

    int sas_ang_idx;
    int sas_spd_idx;
    int whl4_spd_len;
    int lat_acc_idx;
    int lon_acc_idx;
    int yaw_rate_idx;
    int gear_idx;

    double current_kmph;
  public:
    explicit IchthusCANKIAReader(const rclcpp::NodeOptions &);
    ~IchthusCANKIAReader() override;

    bool rx();
    void rxThread();
    void rx_kia_Handler(can_frame_t *);
    void load_dbc();
    void load_dbc(std::vector<uint64_t> &, const std::string &);

    void init_Param();
    void ang_handler(std_msgs::msg::Float64MultiArray);
    void spd_handler(std_msgs::msg::Float64MultiArray);
    void acc_handler(std_msgs::msg::Float64MultiArray);
    void gear_handler(std_msgs::msg::Float64MultiArray);
    void publishOdom();

    rcl_interfaces::msg::SetParametersResult
      param_CB(const std::vector<rclcpp::Parameter> &);

};

}

#endif // ICHTHUS_CAN__KIA_READER_HPP_
