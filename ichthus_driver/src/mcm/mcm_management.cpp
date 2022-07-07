#include <functional>
#include <chrono>

#include "ichthus_driver/SocketCAN.h"
#include "ichthus_driver/mcm_management.hpp"

using namespace std::chrono_literals;


namespace ichthus
{

IchthusCANMCMManager::IchthusCANMCMManager(const rclcpp::NodeOptions & options)
: rclcpp::Node("IchthusCANMCMManager", options)
{
  RCLCPP_INFO(this->get_logger(), "===Start MCM Manager===");
  init_Param();
  state_history_flag = 0;

  adapter = new SocketCAN();
  adapter->open(&*can_interface.begin());

  pidVel_sub = this->create_subscription<ichthus_msgs::msg::Pid>(
    "pid_vel", 10, std::bind(&ichthus::IchthusCANMCMManager::vel_CB,\
                                               this, std::placeholders::_1));

  pidAng_sub = this->create_subscription<ichthus_msgs::msg::Pid>(
    "pid_ang", 10, std::bind(&ichthus::IchthusCANMCMManager::ang_CB,\
                                               this, std::placeholders::_1));

  mcmCtl_sub = this->create_subscription<std_msgs::msg::Int32>(
    "CONTROL_CMD", 1, std::bind(&ichthus::IchthusCANMCMManager::mcm_ctl_CB,\
                                                 this, std::placeholders::_1));

  mcmStatus_pub = this->create_publisher<std_msgs::msg::Int32>("mcm_status", 1);

  mcmStatus_timer = this->create_wall_timer(
    1s, std::bind(&ichthus::IchthusCANMCMManager::mcm_State_Cast, this));

  cb_handle = this->add_on_set_parameters_callback(
    std::bind(&IchthusCANMCMManager::param_CB, this, std::placeholders::_1));
  rx_thread = std::thread(&IchthusCANMCMManager::rxThread, this);
}

IchthusCANMCMManager::~IchthusCANMCMManager()
{
  delete adapter;
  if (rx_thread.joinable())
  {
    RCLCPP_INFO(this->get_logger(), "Thread is joinable");
    std::terminate();
    //rx_thread.join();
  }
  //adapter.reset();
  RCLCPP_INFO(this->get_logger(), "Class Destruct");
}

void IchthusCANMCMManager::init_Param()
{
  this->declare_parameter("can_interface", (std::string)"vcan0");

  can_interface = this->get_parameter("can_interface").as_string();
  setControlToAllFalse(SUBSYS_1);
  setControlToAllFalse(SUBSYS_2);
}

void IchthusCANMCMManager::changeOverrideState(int subsys_id, bool is_override)
{
  if (subsys_id == SUBSYS_1){
    MCM_State_subsys1.OVERRIDE = is_override;
  }else if(subsys_id == SUBSYS_2){
    MCM_State_subsys2.OVERRIDE = is_override;
  }
}

void IchthusCANMCMManager::setControlToAllFalse(int subsys_id){
  if (subsys_id == SUBSYS_1){
    MCM_State_subsys1.Accel_Control_State = false;
    MCM_State_subsys1.Brake_Control_State = false;
    MCM_State_subsys1.Steer_Control_State = false;
  }else if(subsys_id == SUBSYS_2){
    MCM_State_subsys2.Accel_Control_State = false;
    MCM_State_subsys2.Brake_Control_State = false;
    MCM_State_subsys2.Steer_Control_State = false;
  }
}

void IchthusCANMCMManager::vel_CB(const ichthus_msgs::msg::Pid::SharedPtr cmd)
{
  if(get_mcm_State() == MCM_GENERAL_STATE::CONTROL_ENABLED)
  {
    float_hex_convert output;
    can_frame_t send_data;

    if (cmd->frame_id == "Throttle")
    {
      send_data.data[1] = ACCEL_ID;
    }
    else if (cmd->frame_id == "Brake")
    {
      send_data.data[1] = BRAKE_ID;
    }
    output.val = cmd->data;
    adapter->make_can_frame(COMMAND_ID, output, send_data);
    adapter->transmit(send_data);
  }
  /*else
  {
    printf("MCM State Unenable\n");
  }*/
}

void IchthusCANMCMManager::ang_CB(const ichthus_msgs::msg::Pid::SharedPtr cmd)
{
  if(get_mcm_State() == MCM_GENERAL_STATE::CONTROL_ENABLED)
  {
    float_hex_convert output;
    can_frame_t send_data;

    if (cmd->frame_id == "Steer")
    {
      send_data.data[1] = STEER_ID;
    }
    output.val = cmd->data;
    adapter->make_can_frame(COMMAND_ID, output, send_data);
    adapter->transmit(send_data);
  }
}

void IchthusCANMCMManager::mcm_ctl_CB(const std_msgs::msg::Int32::SharedPtr msg)
{
  if(msg->data == 0)          /* Note : disable all interfaces */
  {
    RCLCPP_INFO(this->get_logger(), "All Control Disable");
    adapter->send_control_request(ACCEL_ID, false);
    adapter->send_control_request(BRAKE_ID, false);
    adapter->send_control_request(STEER_ID, false);
    control_mode = 0;
  }
  else if (msg->data == 1)  /*Note : enable all interfaces */
  {
    RCLCPP_INFO(this->get_logger(), "Send All Control Request");
    adapter->send_control_request(ACCEL_ID, true);
    adapter->send_control_request(BRAKE_ID, true);
    adapter->send_control_request(STEER_ID, true);
    control_mode = 1;
  }
  else if (msg->data == 2) /*Note : enable steer only */
  {
    RCLCPP_INFO(this->get_logger(), "Send Steer Control Request");
    adapter->send_control_request(STEER_ID, true);
    control_mode = 2;
  }
  else if (msg->data == 3) /*Note : enable accel, brake only */
  {
    RCLCPP_INFO(this->get_logger(), "Send Accel & Brake Control Request");
    adapter->send_control_request(ACCEL_ID, true);
    adapter->send_control_request(BRAKE_ID, true);
    control_mode = 3;
  }
  sleep(1);
}

MCM_GENERAL_STATE IchthusCANMCMManager::get_mcm_State()
{
  if (MCM_State_subsys1.OVERRIDE || MCM_State_subsys2.OVERRIDE){
    return MCM_GENERAL_STATE::OVERRIDED;
  }
  else if (control_mode == 1)  /* Note : all interface enabled */
  {
    if (MCM_State_subsys1.Brake_Control_State == 1 && \
                MCM_State_subsys1.Accel_Control_State == 1 &&
        MCM_State_subsys1.Steer_Control_State == 1 && \
                MCM_State_subsys2.Brake_Control_State == 1 &&
        MCM_State_subsys2.Accel_Control_State == 1 && \
                MCM_State_subsys2.Steer_Control_State == 1)
    {
      return MCM_GENERAL_STATE::CONTROL_ENABLED;
    }
  }
  else if (control_mode == 2) /* Note : only steer enabled */
  {
    if(MCM_State_subsys1.Steer_Control_State == 1 &&\
                   MCM_State_subsys2.Steer_Control_State == 1)
    {
      return MCM_GENERAL_STATE::CONTROL_ENABLED;
    }
  }
  else if (control_mode == 3) /* Note : only accel, brake enabled */
  {
    if(MCM_State_subsys1.Brake_Control_State == 1 &&\
          MCM_State_subsys1.Accel_Control_State == 1 &&\
          MCM_State_subsys2.Brake_Control_State == 1 &&\
             MCM_State_subsys2.Accel_Control_State == 1)
    {
      return MCM_GENERAL_STATE::CONTROL_ENABLED;
    }
  }
  //To do : convert to false, now because of steer control enable signal
  return MCM_GENERAL_STATE::CONTROL_DISABELD;
}

void IchthusCANMCMManager::rx_mcm_Handler(can_frame_t* frame)
{
  CanMessage::MCM_DATA data;
  switch (frame->can_id)
  {
  case 0x061: //Control Enable Response From MCM
    data.subsys_id = (frame->data[0] & SUBSYS_MASK) >> 7;
    data.hex_id = frame->data[1];
    data.bool_data = frame->data[2];
    data.type = MCM_MESSAGE_TYPE::CONTROL_RESPONSE;
    mcm_State_Update(data);
    break;
  case 0x40: //Override Message 
    if(frame->data[2]){ 
      data.subsys_id = (frame->data[0] & SUBSYS_MASK) >> 7;
      data.hex_id = frame->data[1];
      memcpy(data.override_msg, frame->data+3, sizeof(char)*4);
      data.type = MCM_MESSAGE_TYPE::OVERRIDE;
      mcm_State_Update(data);
    }else if(!frame->data[2]){
      data.subsys_id = (frame->data[0] & SUBSYS_MASK) >> 7;
      data.hex_id = frame->data[1];
      data.type = MCM_MESSAGE_TYPE::OVERRIDE_RESPONSE;
      mcm_State_Update(data);
    }
    break;
  default:
    break;
  }
}


void IchthusCANMCMManager::mcm_State_Update(CanMessage::MCM_DATA data)
{
  if(data.type == MCM_MESSAGE_TYPE::CONTROL_RESPONSE)
  {
    switch (data.hex_id)
    {
    case BRAKE_ID:
      if(data.subsys_id == SUBSYS_1){
        if(MCM_State_subsys1.Brake_Control_State != data.bool_data)
          MCM_State_subsys1.Brake_Control_State = data.bool_data;
      }
      else if(data.subsys_id == SUBSYS_2){
        if(MCM_State_subsys2.Brake_Control_State != data.bool_data)
          MCM_State_subsys2.Brake_Control_State = data.bool_data;      
      }
      printf("MCM [BRAKE] Control state have changed to [SUBSYS1] %d [SUBSYS2] %d \n"\
            , MCM_State_subsys1.Brake_Control_State, MCM_State_subsys2.Brake_Control_State);
      break;
    case ACCEL_ID:
      if(data.subsys_id == SUBSYS_1){
        if(MCM_State_subsys1.Accel_Control_State != data.bool_data)
          MCM_State_subsys1.Accel_Control_State = data.bool_data;
      }
      else if(data.subsys_id == SUBSYS_2){
        if(MCM_State_subsys2.Accel_Control_State != data.bool_data)
          MCM_State_subsys2.Accel_Control_State = data.bool_data;      
      }
      printf("MCM [ACEEL] Control state have changed to [SUBSYS1] %d [SUBSYS2] %d \n"\
            , MCM_State_subsys1.Accel_Control_State, MCM_State_subsys2.Accel_Control_State);
      break;
    case STEER_ID:
      if(data.subsys_id == SUBSYS_1){
        if(MCM_State_subsys1.Steer_Control_State != data.bool_data)
          MCM_State_subsys1.Steer_Control_State = data.bool_data;
      }
      else if(data.subsys_id == SUBSYS_2){
        if(MCM_State_subsys2.Steer_Control_State != data.bool_data)
          MCM_State_subsys2.Steer_Control_State = data.bool_data;      
      }
      printf("MCM [Steer] Control state have changed to [SUBSYS1] %d [SUBSYS2] %d \n"\
            , MCM_State_subsys1.Steer_Control_State, MCM_State_subsys2.Steer_Control_State);
      break;
    default:
      printf("MCM State Update Nothing.\n");
      break;
    }
  }
  else if(data.type == MCM_MESSAGE_TYPE::OVERRIDE)
  {
    setControlToAllFalse(data.subsys_id);
    changeOverrideState(data.subsys_id, true);
    RCLCPP_INFO(this->get_logger(), "MCM Override Occur");
    handleOverrideMsg(data);
  }
  else if(data.type == MCM_MESSAGE_TYPE::FAULT){
    setControlToAllFalse(data.subsys_id);
    RCLCPP_INFO(this->get_logger(), "MCM Fault Occur");
    handleFaultMsg(data);
  }
  else if(data.type == MCM_MESSAGE_TYPE::OVERRIDE_RESPONSE)
  {
    changeOverrideState(data.subsys_id, false);
    RCLCPP_INFO(this->get_logger(), "MCM Override Acked!");
  }
}

void IchthusCANMCMManager::handleOverrideMsg(CanMessage::MCM_DATA data)
{
  ACK_MSG new_msg;
  new_msg.subsys_id = data.subsys_id;
  new_msg.key[0] = data.override_msg[0] ^ ACK_KEY;
  new_msg.key[1] = data.override_msg[1] ^ ACK_KEY;
  new_msg.key[2] = data.override_msg[2] ^ ACK_KEY;
  new_msg.key[3] = data.override_msg[3] ^ ACK_KEY;
  if(override_ack_msg_queue.size() < 2){
    override_ack_msg_queue.push(new_msg);
  }
  if(override_ack_msg_queue.size() == 2){
    overrideAck();
  }
}

void IchthusCANMCMManager::handleFaultMsg(CanMessage::MCM_DATA data)
{
  ACK_MSG new_msg;
  new_msg.subsys_id = data.subsys_id;
  new_msg.key[0] = data.override_msg[0] ^ ACK_KEY;
  new_msg.key[1] = data.override_msg[1] ^ ACK_KEY;
  new_msg.key[2] = data.override_msg[2] ^ ACK_KEY;
  new_msg.key[3] = data.override_msg[3] ^ ACK_KEY;
  
  if(fault_ack_msg_queue.size() < 2){
    fault_ack_msg_queue.push(new_msg);
  }else{
    faultAck();
  }
}

void IchthusCANMCMManager::overrideAck()
{
  while(!override_ack_msg_queue.empty()){
    can_frame_t can_payload;
    ACK_MSG msg = override_ack_msg_queue.front();
    float_hex_convert converter;
    memcpy(converter.data, msg.key, sizeof(char)*4);
    if(msg.subsys_id == SUBSYS_1){
      adapter->make_can_frame(OVERRIDE_ACK, SUBSYS_1_BUS_1, converter, can_payload);
    }
    else if(msg.subsys_id == SUBSYS_2){
      adapter->make_can_frame(OVERRIDE_ACK, SUBSYS_2_BUS_1, converter, can_payload);
    }
    adapter->transmit(can_payload);
    override_ack_msg_queue.pop();
  }
}

void IchthusCANMCMManager::faultAck(){
  /*Not Implemented*/
}

void IchthusCANMCMManager::mcm_State_Cast()
{
  std_msgs::msg::Int32 msg;
  MCM_GENERAL_STATE state = get_mcm_State();
  if (state == MCM_GENERAL_STATE::CONTROL_ENABLED) //if control enabled
  {
    if(state_history_flag != 1){
      state_history_flag = 1;
      msg.data = state; 
      mcmStatus_pub->publish(msg);
    }
  }
  else if (state == MCM_GENERAL_STATE::CONTROL_DISABELD) //if control is disabled
  {
    if(state_history_flag != 2){
      state_history_flag = 2;
      msg.data = state; 
      mcmStatus_pub->publish(msg);
    }
  }
  else if(state == MCM_GENERAL_STATE::OVERRIDED)
  {
    if(state_history_flag != 3){
      state_history_flag = 3;
      msg.data = state;
      mcmStatus_pub->publish(msg);
    }
  }
}

bool IchthusCANMCMManager::rx()
{
  can_frame_t frame;
  if (adapter->receive(frame))
  {
    rx_mcm_Handler(&frame);
    return true;
  }
  else
  {
    return false;
  }
}

void IchthusCANMCMManager::rxThread()
{
  do {
    rx();
  } while (rclcpp::ok());
}

rcl_interfaces::msg::SetParametersResult
IchthusCANMCMManager::param_CB(const std::vector<rclcpp::Parameter> & params)
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
  }
  return result;
}

} //namespace end
