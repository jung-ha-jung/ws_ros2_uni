#include <array>
#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "uni_interface/dataType.h"
#include "uni_interface/modbusMap.h"
#include "uni_interface/systemIO.h"

using namespace std::chrono_literals;

SYSTEM_IO::SYSTEM_IO()
: Node("sio_node")
{
    //get parameters
    this->declare_parameter("robotError_pub_on");
    this->declare_parameter("robotError_pub_topic");

    this->get_parameter_or<bool>("robotError_pub_on", robotError_pub_on, true);
    this->get_parameter_or<std::string>("robotError_pub_topic", robotError_pub_topic, std::string("robot/ErrorMsg"));

    RCLCPP_INFO(this->get_logger(), "get parameter  robotError_pub_on %d", robotError_pub_on);
    RCLCPP_INFO(this->get_logger(), "get parameter  robotError_pub_topic %s", robotError_pub_topic.c_str());

   //initialize pusblisher
    if(robotError_pub_on)
    {
        // Publish the robot error topic
        RCLCPP_INFO(this->get_logger(), "Initialize the publish of robot error topic");
        RobotError_pub_ = this->create_publisher<uni_interface::msg::RobotError>(robotError_pub_topic.c_str(), 10);

        RobotError_msg.source_node = "DIO";
        RobotError_msg.error_type = 0;  // EMG buttion is Release
        RobotError_msg.error_code = 0;
        RobotError_msg.error_msg = "none";
        RobotError_msg.level = "INFO";

    }

    // Publish Emergency Button State
    RCLCPP_INFO(this->get_logger(), "Initialize the publish of robot emergency topic");
    EmgButton_pub_ = this->create_publisher<std_msgs::msg::Bool>("/robot/button/emergency_state", 100);

    MotorPower(1);
    ChargerIo(1);

}


SYSTEM_IO::~SYSTEM_IO()
{
#ifdef _TEST
    ChargerIo(0);
    MotorPower(0);
#else
    ChargerIo(1);
#endif
    RCLCPP_WARN(this->get_logger(), "Exiting systemIO");
}


// ------------------------------------------------------------------
/// @param
/// @return
/// @brief  io 신호 처리
// ------------------------------------------------------------------
void SYSTEM_IO::sig_proc()
{
    RCLCPP_DEBUG(this->get_logger(), "sig_proc()");
    CheckRobotState();
    SystemBehavior();
}


// ------------------------------------------------------------------
/// @param
/// @return
/// @brief  io 메시지 발행
// ------------------------------------------------------------------
void SYSTEM_IO::msg_publisher()
{
    RCLCPP_DEBUG(this->get_logger(), "msg_publisher()");
    systemIO_publisher();
}


// ------------------------------------------------------------------
/// @param
/// @return
/// @brief  에러 메시지 발행
// ------------------------------------------------------------------
void SYSTEM_IO::Error_publish(int error_code)
{
    RCLCPP_DEBUG(this->get_logger(), "Error_publish()");
    if(robotError_pub_on)
    {
        //RobotError_msg.header.stamp = rclcpp::Node::now();
	    switch(error_code){
          case 1:	// Emegency Switch IN
            RobotError_msg.error_type = 200;  // EMG buttion is pressed
            RobotError_msg.error_code = error_code + 199;
            RobotError_msg.error_msg = "DIO: EMG Switch Push";
            RobotError_msg.level = "INFO";
            RCLCPP_DEBUG(this->get_logger(), "DIO: EMG Switch Push");
            break;
          case 2:	// Emegency Switch release
            RobotError_msg.error_type = 201;  // EMG buttion is Release
            RobotError_msg.error_code = error_code + 199;
            RobotError_msg.error_msg = "DIO: EMG Switch Release";
            RobotError_msg.level = "INFO";
            RCLCPP_DEBUG(this->get_logger(), "DIO: EMG Switch Release");
            break;

#ifdef _TEST
          case 100:	// TEST
            RobotError_msg.error_type = 250;  // TEST
            RobotError_msg.error_code = error_code + 199;
            RobotError_msg.error_msg = "DIO: Motor not moving";
            RobotError_msg.level = "ERROR";
            RCLCPP_ERROR(this->get_logger(), "DIO: Motor not moving");
            break;
#endif

          default:
            break;
        }

        RobotError_pub_->publish(RobotError_msg);
    }
}


// ------------------------------------------------------------------
/// @param
/// @return
/// @brief
// ------------------------------------------------------------------
void SYSTEM_IO::systemIO_publisher()
{
    int emergency = get_di(DI_EMERGENCY);
    Emg_msg.data = (emergency > 0)? true : false;

    EmgButton_pub_->publish(Emg_msg);
}


// ------------------------------------------------------------------
/// @param
/// @return
/// @brief
// ------------------------------------------------------------------
void SYSTEM_IO::CheckRobotState()
{
    RCLCPP_DEBUG(this->get_logger(), "SYSTEM_IO::CheckRobotState()");

    // emergency stop
    int emergency = get_di(DI_EMERGENCY);
    if (emergency_prev_ != emergency) {
        if (emergency) // push
            Error_publish(1);
        else // release
            Error_publish(2);
    }
    emergency_prev_ = emergency;

}


// ------------------------------------------------------------------
/// @param
/// @return
/// @brief
// ------------------------------------------------------------------
void SYSTEM_IO::SystemBehavior()
{
    int emergency = get_di(DI_EMERGENCY);
    int on = (emergency == 0)? 1 : 0;
    MotorPower(on);

    ChargerIo(1); // Charger State
}


// ------------------------------------------------------------------
/// @param
/// @return
/// @brief
// ------------------------------------------------------------------
bool SYSTEM_IO::MotorPower(int on)
{
    RCLCPP_DEBUG(this->get_logger(), "MotorPower");
    return set_do(DO_MOT_POWER, on);
}


// ------------------------------------------------------------------
/// @param
/// @return
/// @brief
// ------------------------------------------------------------------
bool SYSTEM_IO::ChargerIo(int on)
{
    RCLCPP_DEBUG(this->get_logger(), "CHARGER IO");
    return set_do(DO_CHARGER_ON, on);
}


// ------------------------------------------------------------------
/// @param
/// @return
/// @brief  DI 입력상태
// ------------------------------------------------------------------
int SYSTEM_IO::get_di(int sig_no)
{
    return get_bit(&DataMem[DI_ADDR_0], sig_no);
}


// ------------------------------------------------------------------
/// @param
/// @return
/// @brief  DO 출력상태
// ------------------------------------------------------------------
int SYSTEM_IO::get_do(int sig_no)
{
    return get_bit(&DataMem[DO_ADDR_0], sig_no);
}


// ------------------------------------------------------------------
/// @param
/// @return
/// @brief  DO 출력
// ------------------------------------------------------------------
int SYSTEM_IO::set_do(int sig_no, int val)
{
    return set_bit(&DataMem[DO_ADDR_0], sig_no, val);
}


// ------------------------------------------------------------------
/// @param
/// @return
/// @brief  DO 출력
// ------------------------------------------------------------------
int SYSTEM_IO::get_bit(IoDataType *data, int idx)
{
    auto word = idx / 16;
    int bit = idx % 16;
    auto us = data[word].u16_data;
    int val = (us >> bit) & 1;

    return val;
}


// ------------------------------------------------------------------
/// @param
/// @return
/// @brief  bit set
// ------------------------------------------------------------------
int SYSTEM_IO::set_bit(IoDataType *data, int idx, int val)
{
    auto word = idx / 16;
    int bit = idx % 16;
    u16 mask = (u16)(1 << bit);
    auto us = data[word].u16_data;
    if(val) us |= mask;
    else us &= ~mask;
    data[word].u16_data = us;

    return 0;
}

