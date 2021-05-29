#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "uni_interface/modbusMap.h"
#include "uni_interface/modbusTCPs.h"
#include "uni_interface/modbusTCPm.h"
#include "uni_interface/can_bat_adl24linmc_39ah_50a_cb.h"
#include "uni_interface/can_mot_sbl2360.h"
#include "uni_interface/systemIO.h"
#include "uni_interface/ros2_uni.h"

using namespace std::chrono_literals;


// ------------------------------------------------------------------
/// @param
/// @return
/// @brief  생성자
// ------------------------------------------------------------------
ros2_uni::ros2_uni():
  Node("ros2_uni_node")
{
    this->declare_parameter("robotError_pub_on");
    this->declare_parameter("robotError_pub_topic");

    this->get_parameter_or<bool>("robotError_pub_on", robotError_pub_on, true);
    RCLCPP_INFO(this->get_logger(),"robotError_pub_on: %d", robotError_pub_on);
    this->get_parameter_or<std::string>("robotError_pub_topic", robotError_pub_topic, "/robot/ErrorMsg");
    RCLCPP_INFO(this->get_logger(),"robotError_pub_topic: %s", robotError_pub_topic.c_str());

    if(robotError_pub_on)
    {
        robotError_pub_ = this->create_publisher<uni_interface::msg::RobotError>(robotError_pub_topic.c_str(), 10);
        robotError_msg.source_node = "INTERFACE";
    }


    ModbusTcpCfgInit(); // modbus slave config init
    ModbusTcpMCfgInit(); // modbus master config init

    // -----------------------------------------
    // CAN_BAT_ADL24 생성
    bat_adl24p_ = new CAN_BAT_ADL24();

    // -----------------------------------------
    // CAN_MOT_SBL2360 생성
    mot_sbl2360p_ = new CAN_MOT_SBL2360();

    // -----------------------------------------
    // SYSTEM_IO 생성
    siop_ = new SYSTEM_IO();


    // task creator -------------------------------------
    task_5ms_ = this->create_wall_timer(
        5ms, std::bind(&ros2_uni::task_5ms, this));

    task_10ms_ = this->create_wall_timer(
        10ms, std::bind(&ros2_uni::task_10ms, this));

    task_100ms_ = this->create_wall_timer(
        100ms, std::bind(&ros2_uni::task_100ms, this));

    task_500ms_ = this->create_wall_timer(
        500ms, std::bind(&ros2_uni::task_500ms, this));

    task_1s_ = this->create_wall_timer(
        1000ms, std::bind(&ros2_uni::task_1s, this));
}


// ------------------------------------------------------------------
/// @param
/// @return
/// @brief  소멸자
// ------------------------------------------------------------------
ros2_uni::~ros2_uni()
{
    RCLCPP_WARN(this->get_logger(), "destory ros2_uni...");
}


// ------------------------------------------------------------------
/// @param
/// @return
/// @brief  task 5ms callback
// ------------------------------------------------------------------
void ros2_uni::task_5ms()
{
    RCLCPP_DEBUG(this->get_logger(), "task_5ms()");

    SetDataMem(9549, GetDataMem(9549).s16_data + 1);
    modbusTcpm_proc(5); // Modbus Master Comm
    // modbusTcps_proc(); // Modbus Slave Comm
    mot_sbl2360p()->cmdvel_loop(); // 모터 cmdvel 생성
}


// ------------------------------------------------------------------
/// @param
/// @return
/// @brief  task 10ms callback
// ------------------------------------------------------------------
void ros2_uni::task_10ms()
{
    RCLCPP_DEBUG(this->get_logger(), "task_10ms()");

    mot_sbl2360p()->msg_publisher(); // 모터 메시지 발행
    siop()->sig_proc(); // io 신호 처리
}


// ------------------------------------------------------------------
/// @param
/// @return
/// @brief  task 100ms callback
// ------------------------------------------------------------------
void ros2_uni::task_100ms()
{
    RCLCPP_DEBUG(this->get_logger(), "task_100ms()");

    bat_adl24p()->check_status(); // 배터리 상태 검사
    siop()->msg_publisher(); // io 메시지 발행
}


// ------------------------------------------------------------------
/// @param
/// @return
/// @brief  task 500ms callback
// ------------------------------------------------------------------
void ros2_uni::task_500ms()
{
    RCLCPP_DEBUG(this->get_logger(), "task_500ms()");

    bat_adl24p()->msg_publisher(); // 배터리 메시지 발행
}


// ------------------------------------------------------------------
/// @param
/// @return
/// @brief  task 1s callback
// ------------------------------------------------------------------
void ros2_uni::task_1s()
{
    RCLCPP_DEBUG(this->get_logger(), "task_1s()");

    mot_sbl2360p()->motorstate_loop(); // 모터 상태 생성
    modbus_comm_check(); // 모드버스 통신 상태 검사
}


// ------------------------------------------------------------------
/// @param
/// @return
/// @brief  modbus comm check
// ------------------------------------------------------------------
void ros2_uni::modbus_comm_check()
{
    int heart_beat = GetDataMem(MODBUS_COMM_HEARTBEAT).s16_data;

    if (heart_beat_prev_ == heart_beat) {
        int error_code = 2;
        if (robotError_pub_on) {
            robotError_msg.error_type = 1001;
            robotError_msg.error_code = error_code + 99;
            robotError_msg.error_msg = "Interface: Modbus Comm Error";
            robotError_msg.level = "ERROR";
            RCLCPP_ERROR(this->get_logger(), "Interface: Modbus Comm Error");
            robotError_pub_->publish(robotError_msg);
        }
    }

    heart_beat_prev_ = heart_beat;
}
