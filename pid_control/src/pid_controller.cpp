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

  #ifdef DEBUG
    DEBUG_pub_str_p_term = this->create_publisher<ichthus_msgs::msg::Common>\
                          ("pid_debug_str_p", 1);
    DEBUG_pub_str_i_term = this->create_publisher<ichthus_msgs::msg::Common>\
                          ("pid_debug_str_i", 1);
    DEBUG_pub_str_d_term = this->create_publisher<ichthus_msgs::msg::Common>\
                          ("pid_debug_str_d", 1);
    DEBUG_pub_str_minimum_term = this->create_publisher<ichthus_msgs::msg::Common>\
                          ("pid_debug_str_minimum", 1);
  #endif



  ref_thr_sub = this->create_subscription<ichthus_msgs::msg::Common>(
    "ref_vel", 10, std::bind(&PIDController::pid_thr_CB, this, std::placeholders::_1));
  ref_str_sub = this->create_subscription<ichthus_msgs::msg::Common>(
    "ref_ang", 10, std::bind(&PIDController::pid_str_CB, this, std::placeholders::_1));

  spd_sub = this->create_subscription<ichthus_msgs::msg::Common>(
    "cur_vel", 10, std::bind(&PIDController::spd_CB, this, std::placeholders::_1));
  ang_sub = this->create_subscription<ichthus_msgs::msg::Common>(
    "cur_ang", 10, std::bind(&PIDController::ang_CB, this, std::placeholders::_1));

  extern_sub = this->create_subscription<std_msgs::msg::Int32>(
    "extern_cmd", 10, std::bind(&PIDController::extern_CB, this, std::placeholders::_1));

  e_stop_sub = this->create_subscription<std_msgs::msg::Bool>(
    "e_stop", 10, std::bind(&PIDController::e_stop_CB, this, std::placeholders::_1));

  imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data", 10, std::bind(&PIDController::imu_CB, this, std::placeholders::_1));
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

  cur_angle_weight = this->declare_parameter("cur_angle_weight", (float)0.1);
  cur_vel_weight = this->declare_parameter("cur_vel_weight", (float)1.0);
  str_max_weight = this->declare_parameter("str_max_weight", (float)0.0);

  max_output_vel = this->declare_parameter("max_vel", (float)0.3);
  max_output_str = this->declare_parameter("max_str", (float)0.35);

  slope_weight = this->declare_parameter("slope_weight", (float)0.11);
  slope_x_coeff = this->declare_parameter("slope_x_coeff", (float)-0.06);

  right_thres = this->declare_parameter("right_thres", (float)0.11);
  left_thres = this->declare_parameter("left_thres", (float)-0.06);

  comfort_time = this->declare_parameter("comfort_time", (float)1.5);
  str_minimum_thrs_buffer = this->declare_parameter("str_minimum_thrs_buffer", (float)1.0);

  use_slope_compensation = this->declare_parameter("use_slope_compensation", false);

  thr_Kp = this->get_parameter("thr_kp").as_double() / PID_CONSTANT;
  thr_Ki = this->get_parameter("thr_ki").as_double() / PID_CONSTANT;
  thr_Kd = this->get_parameter("thr_kd").as_double() / PID_CONSTANT;

  br_Kp = this->get_parameter("br_kp").as_double() / PID_CONSTANT;
  br_Ki = this->get_parameter("br_ki").as_double() / PID_CONSTANT;
  br_Kd = this->get_parameter("br_kd").as_double() / PID_CONSTANT;

  str_Kp = this->get_parameter("str_kp").as_double() / PID_CONSTANT;
  str_Ki = this->get_parameter("str_ki").as_double() / PID_CONSTANT;
  str_Kd = this->get_parameter("str_kd").as_double() / PID_CONSTANT;

  slope_weight = this->get_parameter("slope_weight").as_double();
  slope_x_coeff = this->get_parameter("slope_x_coeff").as_double();

  cur_angle_weight = this->get_parameter("cur_angle_weight").as_double()  / PID_CONSTANT;
  cur_vel_weight = this->get_parameter("cur_vel_weight").as_double()  / PID_CONSTANT;
  str_max_weight = this->get_parameter("str_max_weight").as_double()  / PID_CONSTANT;

  max_output_vel = this->get_parameter("max_vel").as_double() / PID_CONSTANT;
  max_output_str = this->get_parameter("max_str").as_double() / PID_CONSTANT;

  right_thres = this->get_parameter("right_thres").as_double();
  left_thres = this->get_parameter("left_thres").as_double();

  comfort_time = this->get_parameter("comfort_time").as_double();
  use_slope_compensation = this->get_parameter("use_slope_compensation").as_bool();

  str_minimum_thrs_buffer = this->get_parameter("str_minimum_thrs_buffer").as_double();


  hz = 100;
  stop_dt = hz * comfort_time;
  brk_stop_dt = 0;
  start_stopping = true;
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
    else if (param.get_name() =="cur_angle_weight")
    {
      cur_angle_weight = param.as_double();
      RCLCPP_INFO(this->get_logger(), "[PARAM] Change cur_angle_weight : %lf", cur_angle_weight);
    }
    else if (param.get_name() =="cur_vel_weight")
    {
      cur_vel_weight = param.as_double();
      RCLCPP_INFO(this->get_logger(), "[PARAM] Change cur_vel_weight : %lf", cur_vel_weight);
    }
    else if (param.get_name() =="right_thres")
    {
      right_thres = param.as_double();
      RCLCPP_INFO(this->get_logger(), "[PARAM] Change right_thres : %lf", right_thres);
    }
    else if (param.get_name() =="left_thres")
    {
      left_thres = param.as_double();
      RCLCPP_INFO(this->get_logger(), "[PARAM] Change left_thres : %lf", left_thres);
    }
    else if (param.get_name() =="comfort_time")
    {
      comfort_time = param.as_double();
      RCLCPP_INFO(this->get_logger(), "[PARAM] Change comfort_time : %lf", comfort_time);
    }
    else if (param.get_name() =="slope_weight")
    {
      slope_weight = param.as_double();
      RCLCPP_INFO(this->get_logger(), "[PARAM] Change slope_weight : %lf", slope_weight);
    }
    else if (param.get_name() =="slope_x_coeff")
    {
      slope_x_coeff = param.as_double();
      RCLCPP_INFO(this->get_logger(), "[PARAM] Change slope_x_coeff : %lf", slope_x_coeff);
    }
    else if (param.get_name() =="str_max_weight")
    {
      str_max_weight = param.as_double();
      RCLCPP_INFO(this->get_logger(), "[PARAM] Change str_max_weight : %lf", str_max_weight);
    }
    else if (param.get_name() =="str_minimum_thrs_buffer")
    {
      str_max_weight = param.as_double();
      RCLCPP_INFO(this->get_logger(), "[PARAM] Change str_minimum_thrs_buffer : %lf", str_minimum_thrs_buffer);
    }
    else if (param.get_name() =="use_slope_compensation")
    {
      str_max_weight = param.as_double();
      RCLCPP_INFO(this->get_logger(), "[PARAM] Change use_slope_compensation : %d", use_slope_compensation);
    }
  }
  return result;
}

geometry_msgs::msg::Vector3 PIDController::getRPY(geometry_msgs::msg::Quaternion& quat)
{
  geometry_msgs::msg::Vector3 rpy;
  tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf2::Matrix3x3(q).getRPY(rpy.x, rpy.y, rpy.z);
  return rpy;
}

void PIDController::imu_CB(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  auto data = getRPY(msg->orientation);
  //RCLCPP_INFO(this->get_logger(), "SLOPE RAW : %f", data.y);
  cur_slope = data.y;
  //RCLCPP_INFO(this->get_logger(), "SLOPE : %f deg", cur_slope * (180 / 3.14));
}

float PIDController::applySlopeCompensation(float output_before_compensation)
{
  return output_before_compensation + output_before_compensation * slope_weight;
} 

void PIDController::pid_thr_CB(const ichthus_msgs::msg::Common::SharedPtr msg)
{
  ref_vel = msg->data;
  //RCLCPP_INFO(this->get_logger(), "Ref_Vel : %f", msg->data);
}

void PIDController::pid_str_CB(const ichthus_msgs::msg::Common::SharedPtr msg)
{
  ref_ang = -msg->data;
  //RCLCPP_INFO(this->get_logger(), "Ref_Ang : %f", msg->data);
}

void PIDController::spd_CB(const ichthus_msgs::msg::Common::SharedPtr msg)
{

  if (state == pid_state::PID_OFF) {
    return;
  }
  else if (state == pid_state::PID_STANDBY) {
    ichthus_msgs::msg::Pid brk_data;
    brk_data.data = FULL_BRAKE;
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

    brk_data.data = FULL_BRAKE;
    brk_data.frame_id = "Brake";  
    pid_thr_pub->publish(brk_data);
  }
  else if (state == pid_state::PID_ON) {
    ichthus_msgs::msg::Pid acc_data;
    ichthus_msgs::msg::Pid brk_data;
    ichthus_msgs::msg::Pid str_data;
    float vel_err = 0;
    float abs_vel_err = 0;

    cur_vel = msg->data;

    vel_err = ref_vel - cur_vel;
    
    /* set margin wrttien in header */
    /* have to revise set calculate margin */
    //int VEL_BUFFER = getMargine(vel_err, ref_vel);
    float VEL_BUFFER = 1.0;
    if(ref_vel < 0.03){
      #ifdef SMOOTH_BRK_PEDAL /* Note : Push brake pedal to maximum during 1.5s*/
        if (actuation_thr == NO_SIGNAL && start_stopping == true) {
          float remained_brake_percent = 1.0 - (actuation_brk / FULL_BRAKE);
          stop_dt = stop_dt * remained_brake_percent;
          brk_stop_dt = (FULL_BRAKE - actuation_brk) / stop_dt;
          start_stopping = false;
        }
        else if (actuation_brk == NO_SIGNAL && start_stopping == true) {
          acc_data.data = NO_SIGNAL /* For input 0 signal in Data*/;
          acc_data.frame_id = "Throttle";    
          pid_thr_pub->publish(acc_data);
          brk_stop_dt = FULL_BRAKE / stop_dt;
          start_stopping = false;
        }

        actuation_brk += brk_stop_dt;
        if (actuation_brk > FULL_BRAKE) {
          actuation_brk = FULL_BRAKE;
        }

        brk_data.data = actuation_brk;
        brk_data.frame_id = "Brake";  
        pid_thr_pub->publish(brk_data);  
        return;
      #endif // SMOOTH_BRK_PEDAL
      #ifndef SMOOTH_BRK_PEDAL /* Note : Set ref vel -5 when ref_vel < 0.03 km/h
                                        for brake pid */
        ref_vel = -1.8;  /* Note : consider acc/brk buffer */
        vel_err = ref_vel - cur_vel;
      #endif
    }

    if (vel_err < -VEL_BUFFER) //BRK
    {
      acc_data.data = NO_SIGNAL;
      acc_data.frame_id = "Throttle";    
      pid_thr_pub->publish(acc_data);
      brake_pid(abs(vel_err));
      brk_data.data = actuation_brk;
      brk_data.frame_id = "Brake";
      pid_thr_pub->publish(brk_data);
      start_stopping = true;
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
      start_stopping = true;
    }
  }
}

void PIDController::ang_CB(const ichthus_msgs::msg::Common::SharedPtr msg)
{
  if (state == pid_state::PID_OFF) {
    return;
  }
  else if (state == pid_state::PID_STANDBY) {
    return;
  }
  else if (state == pid_state::PID_ON) {
    ichthus_msgs::msg::Pid data;

    float err = 0;
    
    cur_ang = -(msg->data);

    err = ref_ang - cur_ang;
    steer_pid(err);

    data.data = actuation_sas;
    data.frame_id = "Steer";
    pid_str_pub->publish(data);

    //RCLCPP_INFO(this->get_logger(), "error : %f", err);
    //RCLCPP_INFO(this->get_logger(), "thres : %f", threshold);
    //RCLCPP_INFO(this->get_logger(), "angle : %f", cur_ang);
  }
}

void PIDController::throttle_pid(float err)
{
  float vel_err = err;
  float p_term = thr_Kp * vel_err;
  float i_term = thr_Ki * thr_integral;
  float d_term = thr_Kd * (vel_err - thr_velocity_error_last);

  actuation_thr = p_term + i_term + d_term + MINUMIUM_TH;
  if(use_slope_compensation)
    actuation_thr = applySlopeCompensation(actuation_thr);

  if( actuation_thr >= max_output_vel)
  {
    actuation_thr = max_output_vel;
  }
  else if(  actuation_thr <= 0)
  {
    actuation_thr = 0;
  }

  thr_iterm_Lock.lock();
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
  thr_iterm_Lock.unlock();

  thr_velocity_error_last = vel_err; 
}

void PIDController::brake_pid(float err)
{
  float brk_err = err;
  float P, I, D; 
  P = br_Kp * brk_err;
  I = br_Ki * brk_integral;
  D = br_Kd * (brk_err - brk_velocity_error_last);

  actuation_brk = P + I + D;
  if(use_slope_compensation)
    actuation_brk = applySlopeCompensation(actuation_brk);

  if(actuation_brk >= FULL_BRAKE)
  {
      actuation_brk = FULL_BRAKE;
  }
  else if( actuation_brk <= 0)
  {
      actuation_brk = 0;
  }

  thr_iterm_Lock.lock();
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
  thr_iterm_Lock.unlock();

  brk_velocity_error_last = brk_err;
}

void PIDController::steer_pid(float err)
{
  /* TODO: The max output of steer pid is affected by current angle*/
	float P, D, I;   
	float theta, V;
	/* sign: direction of the steer error (+: right, -: left) */
	float sign;
    
	sign = err >= 0.0 ? 1.0 : -1.0;
  
  #ifdef HARDCODE
    if(cur_vel > 30 && abs(err) > 10)
      err = sign * 10;
  #endif

	/* Note: the theta term: 
		 Put additional torque (in Kp),
		 when the direction of the steer error and current steer angle 
	   have the same direction (i.e., ++ or --). 
		 Note: the V term:
		 Put additional torque (in Kp) according to the current velocity (should be positive)
	 */
  theta = cur_ang * cur_angle_weight;
	V = cur_vel_weight*cur_vel*cur_vel;

	P = err * (str_Kp + V);
  #ifdef USE_STR_ITERM
	  I = str_integral * str_Ki;	/* temporaly disable */
	#endif
  D = str_Kd * (err - str_error_last);

  actuation_sas = P + I + D; 
  actuation_sas += theta;
  str_error_last = err;

  #ifdef HARDCODE
    actuation_sas += thres_table(sign, cur_ang, cur_vel);
  #endif

  #ifndef HARDCODE
    if (err > str_minimum_thrs_buffer)  /* Want steer clockwise */
      actuation_sas += (right_thres);
    else if (err < -str_minimum_thrs_buffer)  /* Want steer counter-clockwise */
      actuation_sas += (left_thres);
  #endif

  max_output_str = max_output_str + sign*cur_ang*str_max_weight;
	if (actuation_sas * sign > max_output_str)
		actuation_sas = max_output_str * sign;

  #ifdef DEBUG
    ichthus_msgs::msg::Common debug_p_msg;
    debug_p_msg.header.stamp = this->now();
    debug_p_msg.data = P;
    DEBUG_pub_str_p_term->publish(debug_p_msg);
    ichthus_msgs::msg::Common debug_d_msg;
    debug_d_msg.header.stamp = this->now();
    debug_d_msg.data = theta;
    DEBUG_pub_str_d_term->publish(debug_d_msg);
    ichthus_msgs::msg::Common debug_i_msg;
    debug_i_msg.header.stamp = this->now();
    debug_i_msg.data = I;
    DEBUG_pub_str_i_term->publish(debug_i_msg);
  #endif // DEBUG
  
  #ifdef USE_STR_ITERM
    str_iterm_Lock.lock();
    if(str_iterm_window.size() < MAX_STR_WIN_SIZE){
      str_iterm_window.push_back(err);
      str_integral += err;
    }else{
      float replaced = str_iterm_window.front();
      str_iterm_window.pop_front();
      str_iterm_window.push_back(err);
      str_integral -= replaced;
      str_integral += err;
    }
    str_iterm_Lock.unlock();
  #endif  
  
  RCLCPP_INFO(this->get_logger(), "===\nV_term : %f", V);
  RCLCPP_INFO(this->get_logger(), "T_term : %f\n===", theta);
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

    thr_iterm_Lock.lock();
    thr_integral = 0;
    thr_iterm_window.clear();
    brk_integral = 0;
    brk_iterm_window.clear();
    thr_iterm_Lock.unlock();
    str_iterm_Lock.lock();
    str_integral = 0;
    str_iterm_window.clear();
    str_iterm_Lock.unlock();
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


float PIDController::thres_table(float sign, float cur_ang, \
                                                float cur_vel) {
  /* Note : right default 0.055 
            left default -0.025
  */
  float abs_ang = abs(cur_ang);
  float thres = 0;
  if(cur_vel > 35){
    if (sign > 0) {                        // clockwise
      if (abs_ang < 10)
        thres = right_thres + 0.041;
      else if (abs_ang < 20)
        thres = right_thres + 0.046;
      else if (abs_ang < 30)
        thres = right_thres + 0.051;
      else if (abs_ang < 40)
        thres = right_thres + 0.061;
      else 
        thres = right_thres + 0.071;
    }
    else if (sign < 0) {                   // counter clockwise
      if (abs_ang < 10)
        thres = left_thres - 0.04;
      else if (abs_ang < 20)
        thres = left_thres - 0.045;
      else if (abs_ang < 30)
        thres = left_thres - 0.05;
      else if (abs_ang < 40)
        thres = left_thres - 0.06;
      else 
        thres = left_thres - 0.07;
    }
    return thres;
  }
  if(sign > 0)
    return right_thres;
  else  
    return left_thres;
}

int PIDController::getMargine(float err, float ref_vel){
  float choice = abs(err);
  if(ref_vel <= 10){

  }


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
