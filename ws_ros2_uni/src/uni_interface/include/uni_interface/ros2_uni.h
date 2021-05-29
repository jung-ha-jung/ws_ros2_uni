#pragma once

#include <signal.h>
#include <string>
#include <sstream>
#include "uni_interface/msg/robot_error.hpp"

#define MODBUS_COMM_HEARTBEAT    9046 // MODbus comm. heartbeat

class CAN_BAT_ADL24;
class CAN_MOT_SBL2360;
class SYSTEM_IO;

// ------------------------------------------------------------------
/// @class  ros2_uni
/// @author Ha-Jung, Hyundai Robotics Co., Ltd.
/// @brief
// ------------------------------------------------------------------
class ros2_uni : public rclcpp::Node
{
public:
    ros2_uni();
    virtual ~ros2_uni();

    // 속성 접근자
    CAN_BAT_ADL24* bat_adl24p() const { return bat_adl24p_; }
    CAN_MOT_SBL2360* mot_sbl2360p() const { return mot_sbl2360p_; }
    SYSTEM_IO* siop() const { return siop_; }

protected:
    void task_5ms();
    void task_10ms();
    void task_100ms();
    void task_500ms();
    void task_1s();

    void modbus_comm_check();

private:
    rclcpp::TimerBase::SharedPtr task_5ms_;
    rclcpp::TimerBase::SharedPtr task_10ms_;
    rclcpp::TimerBase::SharedPtr task_100ms_;
    rclcpp::TimerBase::SharedPtr task_500ms_;
    rclcpp::TimerBase::SharedPtr task_1s_;

    CAN_BAT_ADL24* bat_adl24p_ = nullptr;
    CAN_MOT_SBL2360* mot_sbl2360p_ = nullptr;
    SYSTEM_IO* siop_ = nullptr;

    int heart_beat_prev_ = 0;
    bool robotError_pub_on;
    uni_interface::msg::RobotError robotError_msg;
    rclcpp::Publisher<uni_interface::msg::RobotError>::SharedPtr robotError_pub_;
    std::string robotError_pub_topic;

};
