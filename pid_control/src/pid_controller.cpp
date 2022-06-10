#include "pid_control/pid_controller.hpp"

namespace ichthus
{

PIDController::PIDController(const rclcpp::NodeOptions & options)
: rclcpp::Node("PIDController", options)
{
  RCLCPP_INFO(this->get_logger(), "===Start PID Contoller===");
  init_Param();

  cb_handle = this->add_on_set_parameters_callback(
    std::bind(&PIDController::param_CB, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Create Pub & Sub");

  pid_thr_pub = this->create_publisher<ichthus_can_msgs::msg::Pid>("pid_vel", 1);
  pid_str_pub = this->create_publisher<ichthus_can_msgs::msg::Pid>("pid_ang", 1);

  ref_thr_sub = this->create_subscription<std_msgs::msg::Float64>(
    "ref_vel", 10, std::bind(&PIDController::pid_thr_CB, this, std::placeholders::_1));
  ref_str_sub = this->create_subscription<std_msgs::msg::Float64>(
    "ref_ang", 10, std::bind(&PIDController::pid_str_CB, this, std::placeholders::_1));

  spd_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "WHL_SPD11", 10, std::bind(&PIDController::spd_CB, this, std::placeholders::_1));
  ang_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "SAS11", 10, std::bind(&PIDController::ang_CB, this, std::placeholders::_1));

  mcm_status_sub = this->create_subscription<std_msgs::msg::Bool>(
    "mcm_status", 10, std::bind(&PIDController::mcm_status_CB, this, std::placeholders::_1));

  extern_sub = this->create_subscription<std_msgs::msg::Int32>(
    "EXTERN_CMD", 10, std::bind(&PIDController::extern_CB, this, std::placeholders::_1));


  RCLCPP_INFO(this->get_logger(), "acc_max : %f", max_output_vel);
}

PIDController::~PIDController()
{
}

void PIDController::init_Param()
{
  slope_idx = 0;
  mcm_flag = false;

  actuation_thr = 0;
  actuation_brk = 0;
  actuation_sas = 0;

  ref_vel = 0;
  ref_ang = 0;

  thr_velocity_error_last = 0;
  thr_integral = 0;

  brk_velocity_error_last = 0;
  brk_integral = 0;

  str_error_last = 0;
  str_integral = 0;

  thr_Kp = this->declare_parameter("thr_kp", (float)0.01);
  thr_Ki = this->declare_parameter("thr_ki", (float)0.0);
  thr_Kd = this->declare_parameter("thr_kd", (float)0.0);

  br_Kp = this->declare_parameter("br_kp", (float)0.0);
  br_Ki = this->declare_parameter("br_ki", (float)0.0);
  br_Kd = this->declare_parameter("br_kd", (float)0.0);

  str_Kp = this->declare_parameter("str_kp", (float)0.01);
  str_Ki = this->declare_parameter("str_ki", (float)0.0);
  str_Kd = this->declare_parameter("str_kd", (float)0.0);

  max_output_vel = this->declare_parameter("max_vel", (float)0.3);
  max_output_brk = this->declare_parameter("max_brk", (float)0.65);
  max_output_str = this->declare_parameter("max_str", (float)0.35);


  thr_Kp = this->get_parameter("thr_kp").as_double() / PID_CONSTANT;
  thr_Ki = this->get_parameter("thr_ki").as_double() / PID_CONSTANT;
  thr_Kd = this->get_parameter("thr_kd").as_double() / PID_CONSTANT;

  br_Kp = this->get_parameter("br_kp").as_double() / PID_CONSTANT;
  br_Ki = this->get_parameter("br_ki").as_double() / PID_CONSTANT;
  br_Kd = this->get_parameter("br_kd").as_double() / PID_CONSTANT;

  str_Kp = this->get_parameter("str_kp").as_double() / PID_CONSTANT;
  str_Ki = this->get_parameter("str_ki").as_double() / PID_CONSTANT;
  str_Kd = this->get_parameter("str_kd").as_double() / PID_CONSTANT;

  max_output_vel = this->get_parameter("max_vel").as_double() / PID_CONSTANT;
  max_output_brk = this->get_parameter("max_brk").as_double() / PID_CONSTANT;
  max_output_str = this->get_parameter("max_str").as_double() / PID_CONSTANT;
}

rcl_interfaces::msg::SetParametersResult
PIDController::param_CB(const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  for (const auto &param : params)
  {
    if (param.get_name() =="max_vel")
    {
      max_output_vel = param.as_double();
      RCLCPP_INFO(this->get_logger(), "[PARAM] Change max_Vel : %lf", max_output_vel);
    }
    else if (param.get_name() =="max_brk")
    {
      max_output_brk = param.as_double();
      RCLCPP_INFO(this->get_logger(), "[PARAM] Change max_Brk : %lf", max_output_brk);
    }
    else if (param.get_name() =="max_str")
    {
      max_output_str = param.as_double();
      RCLCPP_INFO(this->get_logger(), "[PARAM] Change max_Str : %lf", max_output_str);
    }
    else if (param.get_name() =="thr_kp")
    {
      thr_Kp = param.as_double();
      RCLCPP_INFO(this->get_logger(), "[PARAM] Change thr_Kp : %lf", thr_Kp);
    }
    else if (param.get_name() =="thr_ki")
    {
      thr_Ki = param.as_double();
      RCLCPP_INFO(this->get_logger(), "[PARAM] Change thr_Ki : %lf", thr_Ki);
    }
    else if (param.get_name() =="thr_kd")
    {
      thr_Kd = param.as_double();
      RCLCPP_INFO(this->get_logger(), "[PARAM] Change thr_Kd : %lf", thr_Kd);
    }
    else if (param.get_name() =="br_kp")
    {
      br_Kp = param.as_double();
      RCLCPP_INFO(this->get_logger(), "[PARAM] Change br_Kp : %lf", br_Kp);
    }
    else if (param.get_name() =="br_ki")
    {
      br_Ki = param.as_double();
      RCLCPP_INFO(this->get_logger(), "[PARAM] Change br_Ki : %lf", br_Ki);
    }
    else if (param.get_name() =="br_kd")
    {
      br_Kd = param.as_double();
      RCLCPP_INFO(this->get_logger(), "[PARAM] Change br_Kd : %lf", br_Kd);
    }
    else if (param.get_name() =="str_kp")
    {
      str_Kp = param.as_double();
      RCLCPP_INFO(this->get_logger(), "[PARAM] Change str_Kp : %lf", str_Kp);
    }
    else if (param.get_name() =="str_ki")
    {
      str_Ki = param.as_double();
      RCLCPP_INFO(this->get_logger(), "[PARAM] Change str_Ki : %lf", str_Ki);
    }
    else if (param.get_name() =="str_kd")
    {
      str_Kd = param.as_double();
      RCLCPP_INFO(this->get_logger(), "[PARAM] Change str_Kd : %lf", str_Kd);
    }
  }
  return result;
}

void PIDController::pid_thr_CB(const std_msgs::msg::Float64::SharedPtr msg)
{
  ref_vel = msg->data;
  //RCLCPP_INFO(this->get_logger(), "Ref_Vel : %f", msg->data);
}

void PIDController::pid_str_CB(const std_msgs::msg::Float64::SharedPtr msg)
{
  ref_ang = -msg->data;
  //RCLCPP_INFO(this->get_logger(), "Ref_Ang : %f", msg->data);
}

void PIDController::spd_CB(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
/*
 ** actuation_thr, brk is control input velocity
 ** msg->data size is 8
 ** WHL_SPD_FL, WHL_SPD_FR, WHL_SPD_RL, WHL_SPD_RR
 ** WHL_SPD_AliveCounter_LSB, WHL_SPD_AliveCounter_MSB
 ** WHL_SPD_CheckSum_LSB, WHL_SPD_CheckSum_MSB
 */
{
  ichthus_can_msgs::msg::Pid acc_data;
  ichthus_can_msgs::msg::Pid brk_data;
  ichthus_can_msgs::msg::Pid str_data;
  float cur_vel = 0;
  float vel_err = 0;
  float abs_vel_err = 0;
  int idx = 0;
  int size = 4;

  for (auto whl_spd = msg->data.begin(); idx < size; idx++, whl_spd++)
  {
    cur_vel += *whl_spd;
  }
  cur_vel /= size;
  vel_err = ref_vel - cur_vel;
  abs_vel_err = abs(vel_err);

  /* set margin wrttien in header */
  /* have to revise set calculate margin */
  int VEL_BUFFER = choice_margin(vel_err);

  /* Do Brake When First Start of Vehicle (Drive mode Change) */
  if(ref_vel == -1){
    acc_data.data = NO_SIGNAL /* For input 0 signal in Data*/;
    acc_data.frame_id = "Throttle";    
    pid_thr_pub->publish(acc_data);
    brk_data.data = 0.55;
    brk_data.frame_id = "Brake";  
    pid_thr_pub->publish(brk_data);
  }
  else if (vel_err < -VEL_BUFFER) //BRK
  {
    acc_data.data = NO_SIGNAL;
    acc_data.frame_id = "Throttle";    
    pid_thr_pub->publish(acc_data);
    brake_pid(abs_vel_err);
    brk_data.data = actuation_brk;
    brk_data.frame_id = "Brake";
    pid_thr_pub->publish(brk_data);
  }
  else  //ACC
  { 
    throttle_pid(vel_err);
    brk_data.data = NO_SIGNAL;
    brk_data.frame_id = "Brake";
    pid_thr_pub->publish(brk_data);
    acc_data.data = actuation_thr;
    acc_data.frame_id = "Throttle";
    pid_thr_pub->publish(acc_data);
  }
/*
  RCLCPP_INFO(this->get_logger(), "acc : %f", acc_data.data);
  RCLCPP_INFO(this->get_logger(), "brk : %f", brk_data.data);
  RCLCPP_INFO(this->get_logger(), "acc_iterm : %f", thr_integral);
  RCLCPP_INFO(this->get_logger(), "brk_iterm : %f", brk_integral);
*/
}

void PIDController::ang_CB(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
/*
 ** actuation_sas is control input angle
 ** msg->data size is 5
 ** SAS_Angle, SAS_Speed, SAS_Stat, MsgCount, CheckSum
 */
{
  ichthus_can_msgs::msg::Pid data;
  float cur_ang = 0;
  float cur_vel = 0;
  float tar_vel = 0;
  float err = 0;
  float slope = 0;
  
  auto whl_ang = msg->data.begin();
  cur_ang = *whl_ang; 
  cur_vel = *(++whl_ang);
  cur_ang = -cur_ang;

  err = ref_ang - cur_ang;
  steer_pid(err);

  [&](float cur_err, float cur_ang){
    if(cur_err > 0){        //Steer Clockwise
      if(abs(cur_ang) > 40)
        actuation_sas += 0.13; 
      else 
        actuation_sas += 0.12;
    }else if(cur_err < 0){  //Steer Counter-Clockwise
      if(abs(cur_ang) > 40)
        actuation_sas -= 0.08;
      else  
        actuation_sas -= 0.07;
    }
    if(actuation_sas >= max_output_str){
      actuation_sas = max_output_str;
    }
    else if(actuation_sas <= -max_output_str){
      actuation_sas = -max_output_str;
    }
  }(err, cur_ang);

  data.data = actuation_sas;
  RCLCPP_INFO(this->get_logger(), "actuation_sas : %f", actuation_sas);
  RCLCPP_INFO(this->get_logger(), "err : %f", err);
  data.error = err;
  data.frame_id = "Steer";
  pid_str_pub->publish(data);

}


/*
  if(slope_window.size() < SLOPE_SIZE){
    slope_window.push_back(cur_vel);
    
  }else{
    slope = slope_window.back() - slope_window.front();
    slope /= 0.19;
    RCLCPP_INFO(this->get_logger(), "slope : %f", slope);
    slope_window.pop_front();
    slope_window.push_back(cur_vel);
  }
*/

void PIDController::mcm_status_CB(const std_msgs::msg::Bool::SharedPtr msg)
{
  //MCM Control Enable
  if (msg->data == true)
  {
    mcm_flag = true;
  }
  //MCM Control Disable
  else if (msg->data == false)
  {
    mcm_flag = false;
  }
}

void PIDController::throttle_pid(float err)
{
  float vel_err = err;
  float p_term = thr_Kp * vel_err;
  float i_term = thr_Ki * thr_integral;
  float d_term = thr_Kd * (vel_err - thr_velocity_error_last);

  actuation_thr = p_term + i_term + d_term + MINUMIUM_TH;

  if(actuation_thr >= max_output_vel)
  {
      actuation_thr = max_output_vel;
  }
  else if( actuation_thr <= 0)
  {
      actuation_thr = 0;
  }
  //RCLCPP_INFO(this->get_logger(), "throttle : %f", actuation_thr);
  iterm_Lock.lock();
  /* Add Error Window Logic */
  //RCLCPP_INFO(this->get_logger(), "throttle_int_size : %d", thr_iterm_window.size());
  if(thr_iterm_window.size() < MAX_WIN_SIZE){
    thr_iterm_window.push_back(vel_err);
    thr_integral += vel_err;
  }else{
    float replaced = thr_iterm_window.front();
    thr_iterm_window.pop_front();
    thr_iterm_window.push_back(vel_err);
    thr_integral -= replaced;
    thr_integral += vel_err;
  } 
  //RCLCPP_INFO(this->get_logger(), "throttle_error : %f", err);
  //RCLCPP_INFO(this->get_logger(), "throttle_integral : %f", thr_integral);
  iterm_Lock.unlock();

  thr_velocity_error_last = vel_err; 
}

void PIDController::brake_pid(float err)
{
  float brk_err = err; 
  float p_term = br_Kp * brk_err;
  float i_term = br_Ki * brk_integral;
  float d_term = br_Kd * (brk_err - brk_velocity_error_last);

  actuation_brk = p_term + i_term + d_term;

  if(actuation_brk >= max_output_brk)
  {
      actuation_brk = max_output_brk;
  }
  else if( actuation_brk <= 0)
  {
      actuation_brk = 0;
  }

  iterm_Lock.lock();
  /* Add Error Window Logic */
  if(brk_iterm_window.size() < MAX_WIN_SIZE){
    brk_iterm_window.push_back(brk_err);
    brk_integral += brk_err;
  }else{
    float replaced = brk_iterm_window.front();
    brk_iterm_window.pop_front();
    brk_iterm_window.push_back(brk_err);
    brk_integral -= replaced;
    brk_integral += brk_err;
  }
  iterm_Lock.unlock();
  //RCLCPP_INFO(this->get_logger(), "brake : %f", actuation_brk);
  //RCLCPP_INFO(this->get_logger(), "brake_integral : %f", brk_integral);

  brk_velocity_error_last = brk_err;
}

void PIDController::steer_pid(float err)
{
  float ang_err = err; 
  float p_term = str_Kp * ang_err;
  //float i_term = str_Ki * str_integral;
  float d_term = 0;

  d_term = str_Kd * (ang_err - str_error_last);

  str_error_last = ang_err;


  actuation_sas = p_term + d_term;

/*
  if (ang_err > 0) {
    actuation_sas = p_term + i_term + abs(d_term);
  }
  else {
    actuation_sas = p_term + i_term - abs(d_term);
  }
*/
  /*
  if (ang_err > 10)
    actuation_sas = p_term + i_term + d_term + STR_MINUMUM_TH;
  else if (ang_err < -10)
    actuation_sas = p_term + i_term + d_term - STR_MINUMUM_TH;
  else
    actuation_sas = p_term + i_term + d_term;
  */
  //RCLCPP_INFO(this->get_logger(), "str_err : %f", err);
  //RCLCPP_INFO(this->get_logger(), "== Before ==\nact_sas : %f\n P_term : %f, D_term %f\n", actuation_sas, p_term, d_term);
  if (ang_err > 0) {
    if(actuation_sas >= max_output_str){
      actuation_sas = max_output_str;
    }
    /*
    else if(actuation_sas <= 0.05){
      actuation_sas += 0.05;
    }
    */
  }
  else if (ang_err < 0){
    if(actuation_sas <= -max_output_str){
      actuation_sas = -max_output_str;
    }
    /*
    else if(actuation_sas >= -0.05){
      actuation_sas -= 0.05;
    }
    */
  }

  //RCLCPP_INFO(this->get_logger(), "== After ==\nact_sas : %f\n", actuation_sas);
  str_iterm_Lock.lock();
  /* Add Error Window Logic */
  if(str_iterm_window.size() < MAX_STR_WIN_SIZE){
    str_iterm_window.push_back(ang_err);
    str_integral += ang_err;
  }else{
    float replaced = str_iterm_window.front();
    str_iterm_window.pop_front();
    str_iterm_window.push_back(ang_err);
    str_integral -= replaced;
    str_integral += ang_err;
  }
  //RCLCPP_INFO(this->get_logger(), "Ref_Ang : %f", ref_ang);
  //RCLCPP_INFO(this->get_logger(), "Str_Ang : %f", ref_ang / 13.3);
  //RCLCPP_INFO(this->get_logger(), "=====\nang_err : %f \nact_sas : %f\n", ang_err, actuation_sas);
  str_iterm_Lock.unlock();
}

void PIDController::extern_CB(const std_msgs::msg::Int32::SharedPtr msg)
{
  if(msg->data == 1){
    iterm_Lock.lock();
    thr_integral = 0;
    brk_integral = 0;
    iterm_Lock.unlock();
    str_iterm_Lock.lock();
    str_integral = 0;
    str_iterm_Lock.unlock();
  }
}


int PIDController::choice_margin(float err){
  float choice = abs(err);

  if(choice <= 10){
    return margin_table::CASE_A;
  }else if( choice <= 20){
    return margin_table::CASE_B;
  }else if( choice <= 30){
    return margin_table::CASE_C;
  }else if( choice <= 40){
    return margin_table::CASE_D;
  }else if( choice <= 50){
    return margin_table::CASE_E;
  }else if( choice <= 60){
    return margin_table::CASE_F;
  }else{
    return margin_table::CASE_G; 
  }

}


} //namespace end
