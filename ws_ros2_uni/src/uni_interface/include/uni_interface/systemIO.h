#ifndef SYSTEM_IO_H_
#define SYSTEM_IO_H_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "uni_interface/msg/robot_error.hpp"

#define DELTAT(_nowtime,_thentime) ((_thentime>_nowtime)?((0xffffffff-_thentime)+_nowtime):(_nowtime-_thentime))

#define DI_ADDR_0           9000
#define DO_ADDR_0           9500

/** Output **/
#define DO_MOT_POWER        0 // Motor(1,2), Brake(1,2), 응용부 Switching Power
#define DO_CHARGER_ON       1
#define DO_LED1             2
#define DO_LED2             3
#define DO_LED3             4
#define DO_LED4             5

/** Input **/
#define DI_EMERGENCY        0
#define DI_BUMPER1          1
#define DI_BUMPER2          2
#define DI_BUMPER3          3
#define DI_BUMPER4          4
#define DI_JOYSTICK_ENBL    5
#define DI_CHARGER_ENBL     6

//#define _TEST
#ifdef _TEST
    #include <geometry_msgs/Twist.h>
    #include <gpio_systemio/Duplex.h>
#endif

class SYSTEM_IO : public rclcpp::Node
{
public:
    SYSTEM_IO();
    virtual ~SYSTEM_IO();

    void sig_proc();
    void msg_publisher();

protected:
    void Error_publish(int error_code);
    void systemIO_publisher();
    void CheckRobotState();
    void SystemBehavior();
    bool MotorPower(int on);
    bool ChargerIo(int on);

    int get_di(int sig_no);
    int get_do(int sig_no);
    int set_do(int sig_no, int val);
    int get_bit(IoDataType *data, int idx);
    int set_bit(IoDataType *data, int idx, int val);

private:
    bool robotError_pub_on;
    std::string robotError_pub_topic;

    uni_interface::msg::RobotError RobotError_msg;
    std_msgs::msg::Bool Emg_msg;

    rclcpp::Publisher<uni_interface::msg::RobotError>::SharedPtr RobotError_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr EmgButton_pub_;

    int emergency_prev_ = 0; // 비상정지 이전 상태
};

#endif /* SYSTEM_IO_H_ */
