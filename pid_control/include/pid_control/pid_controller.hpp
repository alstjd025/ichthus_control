#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

/* ros_msg Header */
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
// #include "tier4_autoware_utils/geometry/geometry.hpp"

/* Standard Header */
#include <mutex>
#include <functional>
#include <vector>
#include <deque>
#include <memory>
#include <cmath>

/* User Header */
#include "ichthus_msgs/msg/pid.hpp"
#include "ichthus_msgs/msg/can.hpp"
#include "ichthus_msgs/msg/common.hpp"

#define DEGtoRAD(deg) ((deg) * (0.017453))
#define RADtoDEG(rad) ((rad) / (0.017453))
#define MINUMIUM_TH 0.11
#define FULL_BRAKE 0.55
#define NO_SIGNAL 0 /* For input 0 signal in Data*/
#define MAX_WIN_SIZE 20 /* iterm window maximam size */
#define MAX_STR_WIN_SIZE 10 /* iterm window maximam size */
#define PID_CONSTANT 10000 /*Will devide the loaded parameters*/
#define PREVIOUS_WORK_BRAKE 0 /* worked Brake Signal previously */


//#define HARDCODE  /* Some hardcoded part.. subject to change */
#define DEBUG
#define SMOOTH_BRK_PEDAL
//#define USE_STR_ITERM

enum margin_table{
  CASE_A = 1,
  CASE_B,
  CASE_C,
  CASE_D,
  CASE_E,
  CASE_F,
  CASE_G,
};

/*
 * PID_OFF : 사람 제어 - Reference 갱신하지만 PID 계산 x
 * PID_STANDBY : 오토 제어 - Full Braking 상태, P -> D 기어 변경
 * PID_ON : 오토 제어 - PID 계산
 */
enum pid_state{
 PID_STANDBY = 1,
 PID_ON,
 PID_OFF,
 E_STOP,
};



namespace ichthus
{

class PIDController : public rclcpp::Node
{
  private:
    rclcpp::Publisher<ichthus_msgs::msg::Pid>::SharedPtr pid_thr_pub;
    rclcpp::Publisher<ichthus_msgs::msg::Pid>::SharedPtr pid_str_pub;
    

    #ifdef DEBUG     
      rclcpp::Publisher<ichthus_msgs::msg::Common>::SharedPtr DEBUG_pub_str_p_term;
      rclcpp::Publisher<ichthus_msgs::msg::Common>::SharedPtr DEBUG_pub_str_d_term;
      rclcpp::Publisher<ichthus_msgs::msg::Common>::SharedPtr DEBUG_pub_str_i_term;
      rclcpp::Publisher<ichthus_msgs::msg::Common>::SharedPtr DEBUG_pub_str_minimum_term;
    #endif // DEBUG

    rclcpp::Subscription<ichthus_msgs::msg::Common>::SharedPtr ref_str_sub;
    rclcpp::Subscription<ichthus_msgs::msg::Common>::SharedPtr ref_thr_sub;
    rclcpp::Subscription<ichthus_msgs::msg::Common>::SharedPtr spd_sub;
    rclcpp::Subscription<ichthus_msgs::msg::Common>::SharedPtr ang_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr e_stop_sub;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr extern_sub;

    OnSetParametersCallbackHandle::SharedPtr cb_handle;

    //!< @brief mutex for safe throttle iterm initialize
    std::mutex thr_iterm_Lock;

    //!< @brief mutex for safe brake iterm initialize
    std::mutex str_iterm_Lock;

    //!< @brief deque for throttle iterm 
    std::deque<float> thr_iterm_window; 

    //!< @brief mutex for safe brake iterm initialize
    std::deque<float> brk_iterm_window; 

    //!< @brief mutex for safe steer iterm initialize
    std::deque<float> str_iterm_window;

    //!< @brief calculated actuation value (throttle)
    float actuation_thr;

    //!< @brief calculated actuation value (brake)
    float actuation_brk;

    //!< @brief calculated actuation value (steer)
    float actuation_sas;

    //!< @brief calculated actuation value after slope compensation (throttle)
    float actuation_thr_after_slope;

    //!< @brief calculated actuation value after slope compensation (brake)
    float actuation_brk_after_slope;

    //!< @brief current angle
    float cur_ang;

    //!< @brief current velocity
    float cur_vel;

    //!< @brief reference velocity
    float ref_vel;

    //!< @brief reference steering wheel angle (deg)
    float ref_ang;

    //!< @brief current terrain slope (rad)
    float cur_slope;

    //!< @brief P constant of throttle pid
    float thr_Kp;
    //!< @brief I constant of throttle pid
    float thr_Ki;
    //!< @brief D constant of throttle pid
    float thr_Kd;

    //!< @brief P constant of brake pid
    float br_Kp;
    //!< @brief I constant of brake pid
    float br_Ki;
    //!< @brief D constant of brake pid
    float br_Kd;

    //!< @brief P constant of steer pid
    float str_Kp;
    //!< @brief I constant of steer pid
    float str_Ki;
    //!< @brief D constant of steer pid
    float str_Kd;

    //!< @brief last error used in throttle pid
    float thr_velocity_error_last;

    //!< @brief integral term of throttle pid
    //!  <MUST USE MUTEX>
    float thr_integral;

    //!< @brief last error used in brake pid
    float brk_velocity_error_last;

    //!< @brief integral term of brake pid
    //!  <MUST USE MUTEX>
    float brk_integral;

    //!< @brief angle weight for theta term of steer pid
    //!  [angle pid output : P + D + theta_ + velocity_]
		float cur_angle_weight;	

    //!< @brief velocity weight for velocity term of steer pid
    //!  STR_Kp weight per velocity (Note: (v^2 * str_Kp))
		float cur_vel_weight;

    //!< @brief weight for angle-adaptive maximum of steer pid
    float str_max_weight;

    //!< @brief minimum threshold buffer for steer pid
    float str_minimum_thrs_buffer; 

    //!< @brief integral term of steer pid
    //!  <MUST USE MUTEX>
    float str_integral;

    //!< @brief last error used in steer pid
    float str_error_last;

    //!< @brief maximum output value for throttle pid
    float max_output_vel;

    //!< @brief maximum output value for steer pid
    float max_output_str;

    
    //!< @brief minimum threshold slope value (currently not use)
    float slope_x_coeff;
    
    //!< @brief weight for slope compensation
    //! (output = output + output * slope wieght)
    float slope_weight;

    
    //!< @brief threshold used for clockwise steering
    //!  (this value statically added to steer pid output)
    float right_thres;

    //!< @brief threshold used for counter-clockwise steering
    //!  (this value statically added to steer pid output)
    float left_thres;

    //!< @brief delay time for current brake pedal depth to full brake depth
    float comfort_time;
    
    //!< @brief 
    float hz;

    //!< @brief delay time for current brake pedal depth to full brake depth
    float stop_dt;
    float brk_stop_dt;
    bool start_stopping;

    //!< @brief if true, use slope compensation for throttle, brake pid
    bool use_slope_compensation;

    int state;

    margin_table margin;
  public:
    explicit PIDController(const rclcpp::NodeOptions &);
    ~PIDController() override;

    rcl_interfaces::msg::SetParametersResult
      param_CB(const std::vector<rclcpp::Parameter> &);

    void init_Param();
    void pid_thr_CB(const ichthus_msgs::msg::Common::SharedPtr);
    void pid_str_CB(const ichthus_msgs::msg::Common::SharedPtr);
    void spd_CB(const ichthus_msgs::msg::Common::SharedPtr);
    void ang_CB(const ichthus_msgs::msg::Common::SharedPtr);
    void e_stop_CB(const std_msgs::msg::Bool::SharedPtr);
    void imu_CB(const sensor_msgs::msg::Imu::SharedPtr);
    void throttle_pid(float);
    void brake_pid(float);
    void steer_pid(float);

    void extern_CB(const std_msgs::msg::Int32::SharedPtr);

    void update_ref_spd(float vel);
    void update_ref_ang(float vel);
    float get_ref_ang();
    float get_ref_vel();

    float thres_table(float sign, float cur_ang, float cur_vel);

    int getMargine(float err, float ref_vel);
    float applySlopeCompensation(float output_before_compensation);
    geometry_msgs::msg::Vector3 getRPY(geometry_msgs::msg::Quaternion& quat);


};

}

