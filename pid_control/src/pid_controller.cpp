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

  pid_thr_pub = this->create_publisher<ichthus_msgs::msg::Pid>("pid_vel", 1);
  pid_str_pub = this->create_publisher<ichthus_msgs::msg::Pid>("pid_ang", 1);

  ref_thr_sub = this->create_subscription<std_msgs::msg::Float64>(
    "ref_vel", 10, std::bind(&PIDController::pid_thr_CB, this, std::placeholders::_1));
  ref_str_sub = this->create_subscription<std_msgs::msg::Float64>(
    "ref_ang", 10, std::bind(&PIDController::pid_str_CB, this, std::placeholders::_1));

  spd_sub = this->create_subscription<std_msgs::msg::Float64>(
    "cur_vel", 10, std::bind(&PIDController::spd_CB, this, std::placeholders::_1));
  ang_sub = this->create_subscription<std_msgs::msg::Float64>(
    "cur_ang", 10, std::bind(&PIDController::ang_CB, this, std::placeholders::_1));

  extern_sub = this->create_subscription<std_msgs::msg::Int32>(
    "extern_cmd", 10, std::bind(&PIDController::extern_CB, this, std::placeholders::_1));

  e_stop_sub = this->create_subscription<std_msgs::msg::Bool>(
    "e_stop", 10, std::bind(&PIDController::e_stop_CB, this, std::placeholders::_1));
}

PIDController::~PIDController()
{
}

void PIDController::init_Param()
{
  state = pid_state::PID_OFF;

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
  //str_integral = 0;

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

  right_thres = this->declare_parameter("right_thres", (float)0.11);
  left_thres = this->declare_parameter("left_thres", (float)-0.06);

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

  right_thres = this->get_parameter("right_thres").as_double();
  left_thres = this->get_parameter("left_thres").as_double();
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

void PIDController::spd_CB(const std_msgs::msg::Float64::SharedPtr msg)
{
  if (state == pid_state::PID_OFF) {
    return;
  }
  else if (state == pid_state::PID_STANDBY) {
    ichthus_msgs::msg::Pid brk_data;
    brk_data.data = 0.55;
    brk_data.frame_id = "Brake";  
    pid_thr_pub->publish(brk_data);
  }
  else if (state == pid_state::E_STOP) {
    ichthus_msgs::msg::Pid acc_data;
    ichthus_msgs::msg::Pid brk_data;

    if (actuation_brk > PREVIOUS_WORK_BRAKE) {
      acc_data.data = NO_SIGNAL /* For input 0 signal in Data*/;
      acc_data.frame_id = "Throttle";
      pid_thr_pub->publish(acc_data);
    }

    brk_data.data = 0.55;
    brk_data.frame_id = "Brake";  
    pid_thr_pub->publish(brk_data);
  }
  else if (state == pid_state::PID_ON) {
    ichthus_msgs::msg::Pid acc_data;
    ichthus_msgs::msg::Pid brk_data;
    ichthus_msgs::msg::Pid str_data;
    float cur_vel = 0;
    float vel_err = 0;
    float abs_vel_err = 0;

    cur_vel = msg->data;

    vel_err = ref_vel - cur_vel;
    abs_vel_err = abs(vel_err);

    /* set margin wrttien in header */
    /* have to revise set calculate margin */
    int VEL_BUFFER = choice_margin(vel_err);
    if(ref_vel == 0){
      acc_data.data = NO_SIGNAL /* For input 0 signal in Data*/;
      acc_data.frame_id = "Throttle";    
      pid_thr_pub->publish(acc_data);
      brk_data.data = 0.45;
      brk_data.frame_id = "Brake";  
      pid_thr_pub->publish(brk_data);

    }
    else if(ref_vel == -1){
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
      actuation_brk = NO_SIGNAL;
      brk_data.data = actuation_brk;
      brk_data.frame_id = "Brake";
      pid_thr_pub->publish(brk_data);
      acc_data.data = actuation_thr;
      acc_data.frame_id = "Throttle";
      pid_thr_pub->publish(acc_data);
    }
  }
}

void PIDController::ang_CB(const std_msgs::msg::Float64::SharedPtr msg)
{
  if (state == pid_state::PID_OFF) {
    return;
  }
  else if (state == pid_state::PID_STANDBY) {
    return;
  }
  else if (state == pid_state::PID_ON) {
    ichthus_msgs::msg::Pid data;
    float cur_ang = 0;
    float err = 0;
    float threshold = 0;
    
    cur_ang = msg->data; 
    cur_ang = -cur_ang;

    err = ref_ang - cur_ang;
    steer_pid(err);

    if(err > 0){        //Steer Clockwise
      if(cur_ang >= 0) {
        threshold = thres_table(cur_ang);
        actuation_sas += threshold;
      } 
      else 
        actuation_sas += right_thres;
    }
    else if(err < 0){  //Steer Counter-Clockwise
      if(cur_ang < 0) {
        threshold = thres_table(cur_ang);
        actuation_sas += threshold;
      }
      else  
        actuation_sas += left_thres;
    }

    if(actuation_sas >= max_output_str){
      actuation_sas = max_output_str;
    }
    else if(actuation_sas <= -max_output_str){
      actuation_sas = -max_output_str;
    }

    data.data = actuation_sas;
    data.frame_id = "Steer";
    pid_str_pub->publish(data);
    RCLCPP_INFO(this->get_logger(), "error : %f", err);
    RCLCPP_INFO(this->get_logger(), "thres : %f", threshold);
    RCLCPP_INFO(this->get_logger(), "angle : %f", cur_ang);
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

  acc_iterm_Lock.lock();
  /* Add Error Window Logic */
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
  acc_iterm_Lock.unlock();

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

  acc_iterm_Lock.lock();
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
  acc_iterm_Lock.unlock();

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

  if (ang_err > 0) {
    if(actuation_sas >= max_output_str){
      actuation_sas = max_output_str;
    }
  }
  else if (ang_err < 0){
    if(actuation_sas <= -max_output_str){
      actuation_sas = -max_output_str;
    }
  }
  /*
  str_iterm_Lock.lock();
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
  str_iterm_Lock.unlock();
  */
}

void PIDController::extern_CB(const std_msgs::msg::Int32::SharedPtr msg)
{
  if (msg->data == pid_state::PID_STANDBY) {
    state = msg->data;
  }
  else if (msg->data == pid_state::PID_ON) {
    state = msg->data;
  }
  else if (msg->data == pid_state::PID_OFF) {
    state = msg->data;

    acc_iterm_Lock.lock();
    thr_integral = 0;
    thr_iterm_window.clear();
    brk_integral = 0;
    brk_iterm_window.clear();
    acc_iterm_Lock.unlock();
    /*
    str_iterm_Lock.lock();
    str_integral = 0;
    str_iterm_window.clear();
    str_iterm_Lock.unlock();
    */
  }
}

void PIDController::e_stop_CB(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data == true) {
    state = pid_state::E_STOP;
  }
  else if (msg->data == false) {
    return;
  }
}


float PIDController::thres_table(float ang) {
  float thres = 0;
  if (ang >= 0) {
    thres = ang / X_SLOPE + right_thres;
  }
  else if (ang < 0) {
    thres = ang / X_SLOPE + left_thres;
  }
  return thres;
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
