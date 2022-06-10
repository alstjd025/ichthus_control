#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

/* ros_msg Header */
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

/* Standard Header */
#include <mutex>
#include <functional>
#include <vector>
#include <deque>
#include <memory>
#include <cmath>

/* User Header */
#include "ichthus_can_msgs/msg/pid.hpp"

#define MINUMIUM_TH 0.11
#define LT_STR_MIN_TH 0.05 //It's Right - 0.01
#define RT_STR_MIN_TH 0.09 //It's Right - 0.01
#define NO_SIGNAL 0 /* For input 0 signal in Data*/
#define MAX_WIN_SIZE 20 /* iterm window maximam size */
#define MAX_STR_WIN_SIZE 50 /* iterm window maximam size */
#define PID_CONSTANT 10000 /*Will devide the loaded parameters*/
#define SLOPE_SIZE 19 /* slope window maximam size */


enum margin_table{
  CASE_A = 1,
  CASE_B,
  CASE_C,
  CASE_D,
  CASE_E,
  CASE_F,
  CASE_G,
};
/*속도 변화량에 따라 케이스를 나눌것.(idea update)
  delta_vel<=10km/h margin 1
  delta_vel<=20km/h margin 2
  delta_vel<=30km/h margin 3
  delta_vel<=40km/h margin 4
  delta_vel<=50km/h margin 5
  delta_vel<=60km/h margin 6
  delta_vel<=70km/h margin 7
*/



namespace ichthus
{

class PIDController : public rclcpp::Node
{
  private:
    rclcpp::Publisher<ichthus_can_msgs::msg::Pid>::SharedPtr pid_thr_pub;
    rclcpp::Publisher<ichthus_can_msgs::msg::Pid>::SharedPtr pid_str_pub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr ref_thr_sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr ref_str_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr spd_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ang_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mcm_status_sub;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr extern_sub;

    OnSetParametersCallbackHandle::SharedPtr cb_handle;

    std::mutex ref_ang_Lock;
    std::mutex iterm_Lock;
    std::mutex str_iterm_Lock;

    std::deque<float> thr_iterm_window; // 5 seconds error will accumulated
    std::deque<float> brk_iterm_window; 
    std::deque<float> str_iterm_window; 

    float actuation_thr;
    float actuation_brk;
    float actuation_sas;
    float output_vel;
    float output_ang;
    float ref_vel;
    float ref_ang;

    float thr_Kp;
    float thr_Ki;
    float thr_Kd;

    float br_Kp;
    float br_Ki;
    float br_Kd;

    float str_Kp;
    float str_Ki;
    float str_Kd;

    float thr_velocity_error_last;
    float thr_integral;

    float brk_velocity_error_last;
    float brk_integral;

    float str_error_last;
    float str_integral;

    float max_output_vel;
    float max_output_brk; 
    float max_output_str;
    float max_rate;

    
    std::deque<float> slope_window; 
    int slope_idx;

    bool mcm_flag;

    margin_table margin;
  public:
    explicit PIDController(const rclcpp::NodeOptions &);
    ~PIDController() override;

    rcl_interfaces::msg::SetParametersResult
      param_CB(const std::vector<rclcpp::Parameter> &);

    void init_Param();
    void pid_thr_CB(const std_msgs::msg::Float64::SharedPtr);
    void pid_str_CB(const std_msgs::msg::Float64::SharedPtr);
    void spd_CB(const std_msgs::msg::Float64MultiArray::SharedPtr);
    void ang_CB(const std_msgs::msg::Float64MultiArray::SharedPtr);
    void mcm_status_CB(const std_msgs::msg::Bool::SharedPtr);
    void throttle_pid(float);
    void brake_pid(float);
    void steer_pid(float);

    void extern_CB(const std_msgs::msg::Int32::SharedPtr);

    void update_ref_spd(float vel);
    void update_ref_ang(float vel);
    float get_ref_ang();
    float get_ref_vel();

    int choice_margin(float err);
};

}
