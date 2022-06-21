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
#define LT_STR_MIN_TH 0.05 //It's Right - 0.01
#define RT_STR_MIN_TH 0.09 //It's Right - 0.01
#define NO_SIGNAL 0 /* For input 0 signal in Data*/
#define MAX_WIN_SIZE 20 /* iterm window maximam size */
#define MAX_STR_WIN_SIZE 50 /* iterm window maximam size */
#define PID_CONSTANT 10000 /*Will devide the loaded parameters*/
#define PREVIOUS_WORK_BRAKE 0 /* worked Brake Signal previously */

#define DEBUG

// #define X_SLOPE 11250
//#define X_SLOPE 1000000
// #define IMU_ERROR 2 /**/

/*속도 변화량에 따라 케이스를 나눌것.(idea update)
  delta_vel<=10km/h margin 1
  delta_vel<=20km/h margin 2
  delta_vel<=30km/h margin 3
  delta_vel<=40km/h margin 4
  delta_vel<=50km/h margin 5
  delta_vel<=60km/h margin 6
  delta_vel<=70km/h margin 7
  /imu/data
*/
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

    std::mutex acc_iterm_Lock;
    std::mutex str_iterm_Lock;

    std::deque<float> thr_iterm_window; // 5 seconds error will accumulated
    std::deque<float> brk_iterm_window; 
    //std::deque<float> str_iterm_window; 

    float actuation_thr;
    float actuation_brk;
    float actuation_sas;

    float actuation_thr_after_slope;
    float actuation_brk_after_slope;

    float output_vel;
    float output_ang;
    float ref_vel;
    float ref_ang;

    float cur_slope;

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

		float weight_str_Kp;	/* STR_Kp weight per velocity (Note: Kp(1.0+WK_p*vel)) */
		float base_str_Kp;
		float velocity_last;

    float brk_velocity_error_last;
    float brk_integral;

    float str_error_last;
    //float str_integral;

    float max_output_vel;
    float max_output_brk; 
    float max_output_str;
    float max_rate;

    float slope_x_coeff;
    float imu_error;

    float right_thres;
    float left_thres;

    float comfort_time;
    float hz;
    float stop_dt;
    float brk_stop_dt;
    bool start_stopping;

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

    float thres_table(float ang);

    int getMargine(float err, float ref_vel);
    float applySlopeCompensation(float output_before_comp);
    geometry_msgs::msg::Vector3 getRPY(geometry_msgs::msg::Quaternion& quat);


};

}

