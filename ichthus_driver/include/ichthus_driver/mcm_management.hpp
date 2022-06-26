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
    /*Publishers*/
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr mcmStatus_pub;
  
    /*Subscriptions*/
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mcmCtl_sub;
    rclcpp::Subscription<ichthus_msgs::msg::Pid>::SharedPtr pidVel_sub;
    rclcpp::Subscription<ichthus_msgs::msg::Pid>::SharedPtr pidAng_sub;
    rclcpp::TimerBase::SharedPtr mcmStatus_timer;

    OnSetParametersCallbackHandle::SharedPtr cb_handle;
    
    //! @brief SocketCAN object includes socket discriptor and write, read funcs
    SocketCAN * adapter;

    //! @brief MCM State variable for subsystem 1
    MCM_STATE MCM_State_subsys1;

    //! @brief MCM State variable for subsystem 2
    MCM_STATE MCM_State_subsys2;

    //! @brief can interface name (i.e. CAN0)
    std::string can_interface;

    //! @brief worker thread for reader 
    std::thread rx_thread;
  
    //! @brief can interface name (i.e. CAN0)
    int control_mode;

    //! @brief prev state of MCM
    int state_history_flag;

    //! @brief before transmitted, override ack msg will be saved here 
    std::queue<ACK_MSG> override_ack_msg_queue;

    //! @brief before transmitted, fault ack msg will be saved here 
    std::queue<ACK_MSG> fault_ack_msg_queue;

  public:
    explicit IchthusCANMCMManager(const rclcpp::NodeOptions &);
    ~IchthusCANMCMManager() override;

    //! @brief velocity callback func (sends throttle, brake pid output to mcm)
    void vel_CB(const ichthus_msgs::msg::Pid::SharedPtr);
    
    //! @brief velocity callback func (sends steer pid output to mcm)
    void ang_CB(const ichthus_msgs::msg::Pid::SharedPtr);
    
    //! @brief mcm callback func
    //! sends mcm interface enable/disable signal (command from keyboardcontroller)
    void mcm_ctl_CB(const std_msgs::msg::Int32::SharedPtr);

    //! @brief returns current mcm state 
    //! (because it only returns the temporary saved state of mcm,
    //!  the implicit state can be differ with this, subject to change) 
    MCM_GENERAL_STATE get_mcm_State();

    //! @brief set every control to false.
    //! use only in mcm_State_update
    void setControlToAllFalse(int subsys_id);

    //! @brief change the override state.
    //! use only in mcm_State_update
    void changeOverrideState(int subsys_id, bool is_override);

    //! @brief reflect the mcm state with parsed data from below
    void mcm_State_Update(CanMessage::MCM_DATA);
    
    //! @brief parse recieved data from socketcan(mcm)
    void rx_mcm_Handler(can_frame_t *);
    
    //! @brief timer called func (1s).
    //!  publish current mcm state to keyboardcontroller
    void mcm_State_Cast();
    
    //! @brief used by rx thread
    bool rx();
    
    //! @brief rx worker thread
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
