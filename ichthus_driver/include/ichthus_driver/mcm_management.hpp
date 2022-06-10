#ifndef ICHTHUS_CAN__MCM_MANAGEMENT_HPP_
#define ICHTHUS_CAN__MCM_MANAGEMENT_HPP_  

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <std_msgs/msg/int32.hpp>

#include <unistd.h>

#include <algorithm>
#include <vector>
#include <queue>
#include <string>
#include <unordered_map>
#include <fstream>
#include <memory>
#include <chrono>

#include "ichthus_msgs/msg/pid.hpp"
#include "ichthus_driver/SocketCAN.h"
#include "ichthus_driver/SLCAN.h"
#include "dbcppp/CApi.h"
#include "dbcppp/Network.h"
#include <std_msgs/msg/int32.hpp>


namespace ichthus
{

class IchthusCANMCMManager : public rclcpp::Node
{
  private:
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr mcmStatus_pub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mcmCtl_sub;
    rclcpp::Subscription<ichthus_msgs::msg::Pid>::SharedPtr pidVel_sub;
    rclcpp::Subscription<ichthus_msgs::msg::Pid>::SharedPtr pidAng_sub;
    rclcpp::TimerBase::SharedPtr mcmStatus_timer;

    OnSetParametersCallbackHandle::SharedPtr cb_handle;
    
    //std::shared_ptr<SocketCAN> adapter;
    SocketCAN * adapter;
    MCM_STATE MCM_State_subsys1;
    MCM_STATE MCM_State_subsys2;

    std::string can_interface;
    std::thread rx_thread;
    
    int control_mode;
    int state_history_flag;

    std::queue<ACK_MSG> override_ack_msg_queue;
    std::queue<ACK_MSG> fault_ack_msg_queue;

  public:
    explicit IchthusCANMCMManager(const rclcpp::NodeOptions &);
    ~IchthusCANMCMManager() override;

    // CAN MSG & ROS MSG CALLBACKS & HANDLERS
    void vel_CB(const ichthus_msgs::msg::Pid::SharedPtr);
    void ang_CB(const ichthus_msgs::msg::Pid::SharedPtr);
    void mcm_ctl_CB(const std_msgs::msg::Int32::SharedPtr);
    MCM_GENERAL_STATE get_mcm_State();
    void setControlToAllFalse(int subsys_id);
    void changeOverrideState(int subsys_id, bool is_override);

    void rx_mcm_Handler(can_frame_t *);
    void mcm_State_Update(CanMessage::MCM_DATA);
    void mcm_State_Cast();
    bool rx();
    void rxThread();

    // Fault Handling Functions         
    void handleOverrideMsg(CanMessage::MCM_DATA data);
    void handleFaultMsg(CanMessage::MCM_DATA data);
    void overrideAck();
    void faultAck();

    void init_Param();
    rcl_interfaces::msg::SetParametersResult
      param_CB(const std::vector<rclcpp::Parameter> &);

};

}

#endif // ICHTHUS_CAN__MCM_MANAGEMENT_HPP_
