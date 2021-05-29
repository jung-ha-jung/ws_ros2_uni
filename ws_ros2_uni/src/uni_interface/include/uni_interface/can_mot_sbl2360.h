#ifndef CAN_MOT_SBL2360_H_
#define CAN_MOT_SBL2360_H_

// #include <cstdlib>
// #include <cerrno>
// #include <cstring>
// #include <sys/ioctl.h>
// #include <fcntl.h>
// #include <unistd.h>
#include <chrono>
// #include <sys/time.h>
// #include <time.h>
#include <math.h>
#include <signal.h>
#include <string>
#include <sstream>

// ROS related
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "dataType.h"
#include <uni_interface/msg/duplex.hpp>
#include <uni_interface/msg/robot_error.hpp>
#include <uni_interface/msg/set_encoder.hpp>    // Set right and left encoder value message

#include <uni_interface/srv/request_motor_behavior.hpp> // SrvTutorial 서비스 파일 헤더 (빌드후 자동 생성됨)

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

//#define _PUB_TF

#ifdef _ODOM_COVAR_SERVER
  #include "mot_sbl2360_msgs/msg/odometry_covariances.hpp"
  #include "mot_sbl2360_msgs/msg/request_odometry_covariances.hpp"
#endif

#define DELTAT(_nowtime,_thentime) ((_thentime>_nowtime)?((0xffffffff-_thentime)+_nowtime):(_nowtime-_thentime))

//HUB200
#define LOOP_MODE false  // true: open loop, false: closed loop
#define NORMALIZE(_z) atan2(sin(_z), cos(_z))
#define ROBOT_GEAR_RATIO  1.0
#define ROBOT_BASE  0.3534          //[m]
#define ROBOT_WHEEL_RADIUS  0.065  //[m]
#define ROBOT_ENCODER_PPR 1024   // WEDS5541-A14(500), NME1-UVW-T06(1024)
#define ROBOT_ENCODER_CPR ROBOT_ENCODER_PPR*4  // WEDS5541-A14(2000), NME1-UVW-T06(4096)
#define pi 3.14159265359

#define MOT_CAN_Rx050_0    9030
#define MOT_CAN_Rx051_0    9034
#define MOT_CAN_Rx052_0    9038
#define MOT_CAN_Rx052_3    9041 // CAN comm. error
#define MOT_CAN_Rx053_0    9042

#define MOT_CAN_Tx050_0    9502
#define MOT_CAN_Tx051_0    9506
#define MOT_CAN_Tx052_0    9510
#define MOT_CAN_Tx053_0    9514
#define MOT_CAN_Tx054_0    9518
#define MOT_CAN_Tx055_0    9522
#define MOT_CAN_Tx056_0    9526
#define MOT_CAN_Tx057_0    9530
#define MOT_CAN_Tx058_0    9534
#define MOT_CAN_Tx059_0    9538
#define MOT_CAN_Tx05A_0    9542
#define MOT_CAN_Tx05B_0    9546


class CAN_MOT_SBL2360 : public rclcpp::Node
{
public:
    CAN_MOT_SBL2360();
    virtual ~CAN_MOT_SBL2360();

    void start();
    uint32_t millis();

    void motorbreakoFF();
    void motorbreakoN();
    void EmgStop();
    void EmgRelease();
    void SafetyStop();

    void MOT_GetData0(s16 *ReceiveMSG);
    void MOT_GetData1(s16 *ReceiveMSG);
    void MOT_GetData2(s16 *ReceiveMSG);
    void MOT_GetData3(s16 *ReceiveMSG);
    void Checkfaultflags(uint16_t faultflagsdata);
    void Checkruntimestatus_right(uint16_t runtimestatusflags_rightdata);
    void Checkruntimestatus_left(uint16_t runtimestatusflags_leftdata);
    void Checkstatusflags(uint16_t statusflagsdata);

    void statusoutdebugging();
    // cmd_vel subscriber
    void cmdvel_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg);

    // emgbutton subscriber
    void emgbutton_callback(const std_msgs::msg::Bool::SharedPtr emgbutton_msg);

    // Set encoder subscriber callback function
    void set_encoder_callback(const uni_interface::msg::SetEncoder::SharedPtr set_encoder_msg);

    void cmdvel_setup();
    void write_configure1();
    void write_configure2();
    void write_configure3();
    void write_configure4();
    void write_configure5();
    void write_configure6();
    void write_configure7();
    void write_configure8();

    void cmdvel_loop();
    void motorstate_loop();
    void cmdvel_run();

    // odom publisher
    void publisher_init();

    void odom_Init();
    void odom_setup();
    void odom_loop();
    void odom_ms_run();
    void motor_current_publish();
    void motor_encoder_publish();
    void motor_power_publish();
    void motor_Error_publish(int error_code);
    void odom_publish();

    void msg_publisher();

#ifdef _ODOM_COVAR_SERVER
  void odom_covar_callback(const mot_sbl2360_msgs::RequestOdometryCovariancesRequest& req, mot_sbl2360_msgs::RequestOdometryCovariancesResponse& res);
#endif

protected:
    // cmd_vel subscriber
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdvel_sub_;
    std::string cmdvel_topic;
    int32_t cmd_vel_quesize;

    int32_t right_rpm = 0;
    int32_t left_rpm = 0;

    // emgbutton subscriber
    bool emgbutton_sub_on;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emgbutton_sub_;
    std::string emgbutton_sub_topic;

    // Set encoder subscriber
    rclcpp::Subscription<uni_interface::msg::SetEncoder>::SharedPtr set_encoder_sub_;

#ifdef _PUB_TF
    // odom publisher
    geometry_msgs::TransformStamped tf_msg;
    tf::TransformBroadcaster odom_broadcaster;
#endif

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    nav_msgs::msg::Odometry odom_msg;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::string odom_topic;
    bool odom_pub_on;

    bool encoder_pub_on;
    uni_interface::msg::Duplex encoder_msg;
    rclcpp::Publisher<uni_interface::msg::Duplex>::SharedPtr encoder_pub_;
    std::string encoder_pub_topic;
    bool encoder_count_relative;  // true: relative conuter, false: absolute counter

    bool current_pub_on;
    uni_interface::msg::Duplex current_msg;
    rclcpp::Publisher<uni_interface::msg::Duplex>::SharedPtr current_pub_;
    std::string current_pub_topic;

    bool power_pub_on;
    uni_interface::msg::Duplex power_msg;
    rclcpp::Publisher<uni_interface::msg::Duplex>::SharedPtr power_pub_;
    std::string power_pub_topic;

    bool robotError_pub_on;
    uni_interface::msg::RobotError robotError_msg;
    rclcpp::Publisher<uni_interface::msg::RobotError>::SharedPtr robotError_pub_;
    std::string robotError_pub_topic;

    rclcpp::TimerBase::SharedPtr timer1_;

    //#define	SBL2360_CANID_0		0x050
    int32_t odom_encoder_right = 0;
    int32_t odom_encoder_left = 0;
    int32_t odom_encoder_right_prev = 0;
    int32_t odom_encoder_left_prev = 0;

    //#define	SBL2360_CANID_1		0x051
    int16_t Amp_right, Amp_left;
    int16_t right_power = 0;
    int16_t left_power = 0;

    //#define	SBL2360_CANID_3		0x053
    uint16_t faultflags;
    bool bOverheat;                         // bit 0
	bool bOvervoltage;			            // bit 1
	bool bUndervoltage;			            // bit 2
    bool bShortcircuit;                     // bit 3
    bool bEmergencystop;                    // bit 4
    bool bSetupfault;                       // bit 5
    bool bMOSFETfailure;                    // bit 6
    bool bDefaultconfigurationloaded;       // bit 7

    uint16_t runtimestatusflags_right;
    bool bAmpsLimitactive_right;            // bit 0
	bool bMotorstalled_right;			    // bit 1
	bool bLoopError_right;			        // bit 2
    bool bSafetyStop_right;                 // bit 3
    bool bForwardLimit_right;               // bit 4
    bool bReverseLimit_right;               // bit 5
    bool bAmpsTrigger_right;                // bit 6

    uint16_t runtimestatusflags_left;
    bool bAmpsLimitactive_left;            // bit 0
	bool bMotorstalled_left;			    // bit 1
	bool bLoopError_left;			        // bit 2
    bool bSafetyStop_left;                  // bit 3
    bool bForwardLimit_left;                // bit 4
    bool bReverseLimit_left;                // bit 5
    bool bAmpsTrigger_left;                 // bit 6

    uint16_t statusflags;
    bool bSerialMode;                       // bit 0
	bool bPulsemode;			            // bit 1
	bool bAnalogmode;			            // bit 2
    bool bPowerstageoff;                    // bit 3
    bool bStalldetected;                    // bit 4
    bool bAtlimit;                          // bit 5
    bool bUnused;                           // bit 6
    bool bMicroBasicscriptrunning;          // bit 7
    bool bMotorSensorTunningmode;           // bit 8

    //#define	SBL2360_CANID_11	0x05b
    int32_t set_encoder_right_value;
    int32_t set_encoder_left_value;

    float odom_x = 0.0;
    float odom_y = 0.0;
    float odom_yaw = 0.0;
    float odom_last_x = 0.0;
    float odom_last_y = 0.0;
    float odom_last_yaw = 0.0;
    uint32_t odom_last_time = 0.0;


    //uint16_t motor_mode;      // 0: normal, 1: emg
    bool Motor_Emergencystop_cmd;

    // settings
    std::string odom_frame;
    std::string base_frame;

    bool open_loop = LOOP_MODE;
    double wheel_radius = ROBOT_WHEEL_RADIUS;
    double wheel_circumference = ROBOT_WHEEL_RADIUS;
    double track_width = ROBOT_BASE;
    double gear_ratio = ROBOT_GEAR_RATIO;
    int encoder_cpr = 0;
    int torque_constant;
    int numof_pole_pairs;
    int operating_mode, swap_winding;
    int mot_amps_limit, max_speed_command;
    int max_acceleration_rate, max_deceleration_rate, fault_deceleration_rate;

    int rightwheel_gain_p, rightwheel_gain_i, rightwheel_gain_d;
    int leftwheel_gain_p, leftwheel_gain_i, leftwheel_gain_d;
    int mot_direction_R, mot_direction_L;
    int encoder_ppr = 0;
    int pwm_frequency;

    int auto_sending_rate_ms;
    uint32_t auto_sending_rate_ms_cnt;
};

#endif /* CAN_MOT_SBL2360_H_ */
