#ifndef CAN_BAT_ADL24_H_
#define CAN_BAT_ADL24_H_
#include <signal.h>
#include <string>
#include <sstream>

//#include <can_bat_adl24linmc_39ah_50a_cb/ServiceBatteryChargerBehavior.h>

//message include
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "uni_interface/msg/robot_error.hpp"

#define DELTAT(_nowtime,_thentime) ((_thentime>_nowtime)?((0xffffffff-_thentime)+_nowtime):(_nowtime-_thentime))
#define MAX_CHARGER_PROFILEOFF_TIME 60000

#define BAT_CAN_Rx200_0    9002
#define BAT_CAN_Rx201_0    9006
#define BAT_CAN_Rx201_3    9009 // CAN comm. error
#define BAT_CAN_Rx202_0    9010
#define BAT_CAN_Rx203_0    9014
#define BAT_CAN_Rx204_0    9018
#define BAT_CAN_Rx205_0    9022
#define BMS_CAN_Rx070_0    9026

class CAN_BAT_ADL24 : public rclcpp::Node
{
public:
    CAN_BAT_ADL24();
    virtual ~CAN_BAT_ADL24();

    void battery_setup();
    void battery_publisher();
    void check_battery_state();
    void check_charging();
    void debugging_out();
    void battery_Error_publish(int error_code);
    //bool BatteryChargerCallback(can_bat_adl24linmc_39ah_50a_cb::ServiceBatteryChargerBehavior::Request &req, can_bat_adl24linmc_39ah_50a_cb::ServiceBatteryChargerBehavior::Response &res);

    void check_status();
    void msg_publisher();

protected:
	void BATTERY_GetMsg1(s16 *ReceiveMSG);
	void BATTERY_GetMsg2(s16 *ReceiveMSG);
	void BATTERY_GetMsg3(s16 *ReceiveMSG);
	void BMS_GetMsg(s16 *ReceiveMSG);

private:
	// WST Battery
	float voltage = 0.0;		    //
	float voltage_scale = 0.1;	    // 0.1V
    float charge_current = 0.0;		// A
	float discharge_current = 0.0;	// A
	float current_scale = 0.1;		// 0.1A
	float SOC = 0.0;	    		// %, ratio between remaining capacity and the full-capacity
	float remaining_time = 0.0;		// h
	float remaining_time_scale = 0.1;	// 0.1h

	float remaining_capacity = 0.0;	// 1mAh
    float SOH = 0.0;				// %, full-cap/design-cap
	float BMS_ver = 0.0;			//
	float BMS_ver_scale = 0.1;		// 0.1
	float Full_Capacity = 0.0;		// 1mah

	bool charge_state = false;
	bool battery_Over_voltage = false;			// bit 2
	bool battery_Under_voltage = false;			// bit 3
	bool battery_Charge_Over_Current = false;	// bit 4
	bool battery_Discharge_Over_Current = false;// bit 5
	bool battery_Discharge_Over_Temp = false;	// bit 6
	bool battery_Discharge_Under_Temp = false;	// bit 7
	bool battery_Charge_Over_Temp = false;		// bit 10
	bool battery_Charge_Under_Temp = false;		// bit 11

	// RoboteQ BMS
	float bms_loadvoltage;
	float bms_chargervoltage;
	float bms_chargercurrent;
	bool bms_chargerstatus;
	bool bms_chargercmd;

private:
    //float voltage;
    //float ampere;
    //float percentage;
    //bool charge_state;              // charge_state = false
    bool battery_error_state = false;


    // bool auto_charging;             // auto_charging = true;
    // bool auto_discharging;          // auto_discharging = true;
    // double level_to_fullcharging;   // [%]  level_to_fullcharging=100.0;
    // double level_to_autocharging;   // [%]  level_to_autocharging=80.0;
    double level_to_charge = 20.0;     // [%]  level_to_charge = 20.0;

    // bool full_charging_on;          // full_charging_on = false;
    //bool bcharging_start;           // bcharging_start = false;
    //bool exit_chargingstation_on;   // exit_chargingstation_on=true;
    //bool on_chargingstation_on;     // on_chargingstation_on = false;
    //bool switch_start;
    //bool restart_on;

    // battery
    std_msgs::msg::Float64 Voltage_msg;
    bool Bat_vol_pub_on;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr Bat_vol_pub_;
    std::string Bat_vol_pub_topic;

    std_msgs::msg::Float64 charging_Ampere_msg;
    bool Bat_charging_amp_pub_on;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr Bat_charging_amp_pub_;
    std::string Bat_charging_amp_pub_topic;

    std_msgs::msg::Float64 discharging_Ampere_msg;
    bool Bat_discharging_amp_pub_on;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr Bat_discharging_amp_pub_;
    std::string Bat_discharging_amp_pub_topic;

    std_msgs::msg::Float64 Percentage_msg;
    bool Bat_per_pub_on;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr Bat_per_pub_;
    std::string Bat_per_pub_topic;

    std_msgs::msg::Bool Charge_state_msg;
    bool Bat_state_pub_on;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Bat_state_pub_;
    std::string Bat_state_pub_topic;

    // bms
    std_msgs::msg::Float64 bms_loadvoltage_msg;
    std_msgs::msg::Float64 bms_loadvoltage_msgold;
    bool bms_loadvoltage_pub_on;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr bms_loadvoltage_pub_;
    std::string bms_loadvoltage_pub_topic;

    std_msgs::msg::Float64 bms_chargervoltage_msg;
    bool bms_chargervoltage_pub_on;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr bms_chargervoltage_pub_;
    std::string bms_chargervoltage_pub_topic;

    std_msgs::msg::Float64 bms_chargercurrent_msg;
    bool bms_chargercurrent_pub_on;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr bms_chargercurrent_pub_;
    std::string bms_chargercurrent_pub_topic;

    // exception
    bool charge_exception_msg_on;   // charge_exception_msg_on = true;
    bool robotError_pub_on;

    uni_interface::msg::RobotError robotError_msg;
    rclcpp::Publisher<uni_interface::msg::RobotError>::SharedPtr robotError_pub_;
    std::string robotError_pub_topic;

};

#endif /* CAN_BAT_ADL24_H_ */
