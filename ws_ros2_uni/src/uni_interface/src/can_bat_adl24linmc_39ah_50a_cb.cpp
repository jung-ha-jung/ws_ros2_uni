#include <string>
#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "uni_interface/modbusMap.h"
#include "uni_interface/can_bat_adl24linmc_39ah_50a_cb.h"

using namespace std::chrono_literals;

CAN_BAT_ADL24::CAN_BAT_ADL24():
    Node("can_bat_node")
{
    //Parameter declare
    this->declare_parameter("Bat_vol_pub_on");
    this->declare_parameter("Bat_vol_pub_topic");
    this->declare_parameter("Bat_charging_amp_pub_on");
    this->declare_parameter("Bat_charging_amp_pub_topic");
    this->declare_parameter("Bat_discharging_amp_pub_on");
    this->declare_parameter("Bat_discharging_amp_pub_topic");
    this->declare_parameter("Bat_per_pub_on");
    this->declare_parameter("Bat_per_pub_topic");
    this->declare_parameter("Bat_state_pub_on");
    this->declare_parameter("Bat_state_pub_topic");
    this->declare_parameter("bms_loadvoltage_pub_on");
    this->declare_parameter("bms_loadvoltage_pub_topic");
    this->declare_parameter("bms_chargervoltage_pub_on");
    this->declare_parameter("bms_chargervoltage_pub_topic");
    this->declare_parameter("bms_chargercurrent_pub_on");
    this->declare_parameter("bms_chargercurrent_pub_topic");
    this->declare_parameter("robotError_pub_on");
    this->declare_parameter("robotError_pub_topic");
    this->declare_parameter("charge_exception_msg_on");
    this->declare_parameter("level_to_charge");


    // CBA Read local params (from launch file)
    this->get_parameter_or<bool>("Bat_vol_pub_on", Bat_vol_pub_on, true);
    RCLCPP_INFO(this->get_logger(), "Bat_vol_pub_on: %d",  Bat_vol_pub_on);
    this->get_parameter_or<std::string>("Bat_vol_pub_topic", Bat_vol_pub_topic, "/robot/battery/voltage");
    RCLCPP_INFO(this->get_logger(), "Bat_vol_pub_topic: %s",  Bat_vol_pub_topic.c_str());

    this->get_parameter_or<bool>("Bat_charging_amp_pub_on", Bat_charging_amp_pub_on, true);
    RCLCPP_INFO(this->get_logger(), "Bat_charging_amp_pub_on: %d",  Bat_charging_amp_pub_on);
    this->get_parameter_or<std::string>("Bat_charging_amp_pub_topic", Bat_charging_amp_pub_topic, "/robot/battery/charging_ampere");
    RCLCPP_INFO(this->get_logger(), "Bat_charging_amp_pub_topic: %s", Bat_charging_amp_pub_topic.c_str());

    this->get_parameter_or<bool>("Bat_discharging_amp_pub_on", Bat_discharging_amp_pub_on, true);
    RCLCPP_INFO(this->get_logger(), "Bat_discharging_amp_pub_on: %d", Bat_discharging_amp_pub_on);
    this->get_parameter_or<std::string>("Bat_discharging_amp_pub_topic", Bat_discharging_amp_pub_topic, "/robot/battery/discharging_ampere");
    RCLCPP_INFO(this->get_logger(), "Bat_discharging_amp_pub_topic: %s", Bat_discharging_amp_pub_topic.c_str());

    this->get_parameter_or<bool>("Bat_per_pub_on", Bat_per_pub_on, true);
    RCLCPP_INFO(this->get_logger(), "Bat_per_pub_on: %d", Bat_per_pub_on);
    this->get_parameter_or<std::string>("Bat_per_pub_topic", Bat_per_pub_topic, "/robot/battery/percentage");
    RCLCPP_INFO(this->get_logger(), "Bat_per_pub_topic: %s", Bat_per_pub_topic.c_str());

    this->get_parameter_or<bool>("Bat_state_pub_on", Bat_state_pub_on, true);
    RCLCPP_INFO(this->get_logger(),"Bat_state_pub_on: %d", Bat_state_pub_on);
    this->get_parameter_or<std::string>("Bat_state_pub_topic", Bat_state_pub_topic, "/robot/battery/charge_state");
    RCLCPP_INFO(this->get_logger(),"Bat_state_pub_topic: %s", Bat_state_pub_topic.c_str());

    this->get_parameter_or<bool>("bms_loadvoltage_pub_on", bms_loadvoltage_pub_on, true);
    RCLCPP_INFO(this->get_logger(),"bms_loadvoltage_pub_on: %d", bms_loadvoltage_pub_on);
    this->get_parameter_or<std::string>("bms_loadvoltage_pub_topic", bms_loadvoltage_pub_topic, "/robot/bms/loadvoltage");
    RCLCPP_INFO(this->get_logger(),"bms_loadvoltage_pub_topic: %s", bms_loadvoltage_pub_topic.c_str());

    this->get_parameter_or<bool>("bms_chargervoltage_pub_on", bms_chargervoltage_pub_on, true);
    RCLCPP_INFO(this->get_logger(),"bms_chargervoltage_pub_on: %d", bms_chargervoltage_pub_on);
    this->get_parameter_or<std::string>("bms_chargervoltage_pub_topic", bms_chargervoltage_pub_topic, "/robot/bms/chargervoltage");
    RCLCPP_INFO(this->get_logger(),"bms_chargervoltage_pub_topic: %s", bms_chargervoltage_pub_topic.c_str());

    this->get_parameter_or<bool>("bms_chargercurrent_pub_on", bms_chargercurrent_pub_on, true);
    RCLCPP_INFO(this->get_logger(),"bms_chargercurrent_pub_on: %d", bms_chargercurrent_pub_on);
    this->get_parameter_or<std::string>("bms_chargercurrent_pub_topic", bms_chargercurrent_pub_topic, "/robot/bms/chargercurrent");
    RCLCPP_INFO(this->get_logger(),"bms_chargercurrent_pub_topic: %s", bms_chargercurrent_pub_topic.c_str());

    this->get_parameter_or<bool>("robotError_pub_on", robotError_pub_on, true);
    RCLCPP_INFO(this->get_logger(),"robotError_pub_on: %d", robotError_pub_on);
    this->get_parameter_or<std::string>("robotError_pub_topic", robotError_pub_topic, "/robot/ErrorMsg");
    RCLCPP_INFO(this->get_logger(),"robotError_pub_topic: %s", robotError_pub_topic.c_str());

    this->get_parameter_or<bool>("charge_exception_msg_on", charge_exception_msg_on, true);
    RCLCPP_INFO(this->get_logger(),"charge_exception_msg_on: %d", charge_exception_msg_on);

    this->get_parameter_or<double>("level_to_charge", level_to_charge, 20.0);
    RCLCPP_INFO(this->get_logger(),"level_to_charge: %lf", level_to_charge);

    RCLCPP_INFO(this->get_logger(),"Beginning setup Battery");
    battery_setup();
}


//100msec timer callback
void CAN_BAT_ADL24::check_status()
{
    RCLCPP_DEBUG(this->get_logger(), "bat::check_status()");
    int comm_error = GetDataMem(BAT_CAN_Rx201_3).s16_data;

    check_battery_state();
    check_charging();

    if (comm_error)
        battery_Error_publish(2);
}

//500msec timer callback
void CAN_BAT_ADL24::msg_publisher()
{
    RCLCPP_DEBUG(this->get_logger(), "msg_publisher()");

    BATTERY_GetMsg1((s16 *)&DataMem[BAT_CAN_Rx200_0]);
    BATTERY_GetMsg2((s16 *)&DataMem[BAT_CAN_Rx201_0]);
    BATTERY_GetMsg3((s16 *)&DataMem[BAT_CAN_Rx202_0]);
    BMS_GetMsg((s16 *)&DataMem[BMS_CAN_Rx070_0]);

    battery_publisher();
}

CAN_BAT_ADL24::~CAN_BAT_ADL24()
{
    //forceChargerDisable();
    RCLCPP_WARN(this->get_logger(), "Exiting Battery");
}

void CAN_BAT_ADL24::battery_setup()
{
    // Publisher
    if(Bat_vol_pub_on)
        Bat_vol_pub_ = this->create_publisher<std_msgs::msg::Float64>(Bat_vol_pub_topic.c_str(), 10);

    if(Bat_charging_amp_pub_on)
        Bat_charging_amp_pub_ = this->create_publisher<std_msgs::msg::Float64>(Bat_charging_amp_pub_topic.c_str(), 10);

    if(Bat_discharging_amp_pub_on)
        Bat_discharging_amp_pub_ = this->create_publisher<std_msgs::msg::Float64>(Bat_discharging_amp_pub_topic.c_str(), 10);

    if(Bat_per_pub_on)
        Bat_per_pub_ = this->create_publisher<std_msgs::msg::Float64>(Bat_per_pub_topic.c_str(), 10);

    if(Bat_state_pub_on)
        Bat_state_pub_ = this->create_publisher<std_msgs::msg::Bool>(Bat_state_pub_topic.c_str(), 10);

    if(bms_loadvoltage_pub_on)
        bms_loadvoltage_pub_ = this->create_publisher<std_msgs::msg::Float64>(bms_loadvoltage_pub_topic.c_str(), 10);

    if(bms_chargervoltage_pub_on)
        bms_chargervoltage_pub_ = this->create_publisher<std_msgs::msg::Float64>(bms_chargervoltage_pub_topic.c_str(), 10);

    if(bms_chargercurrent_pub_on)
        bms_chargercurrent_pub_ = this->create_publisher<std_msgs::msg::Float64>(bms_chargercurrent_pub_topic.c_str(), 10);

    // if(bms_chargingcmd_pub_on)
    //     bms_chargingcmd_pub_ = nh_.advertise<std_msgs::Bool>(bms_chargingcmd_pub_topic, 10);

    if(robotError_pub_on)
    {
        robotError_pub_ = this->create_publisher<uni_interface::msg::RobotError>(robotError_pub_topic.c_str(), 10);
        robotError_msg.source_node = "BATTERY";
    }
}


void CAN_BAT_ADL24::check_charging()
{

    if(charge_state == true && bms_chargercurrent > 0 )
    {
        if(charge_state == false)
        {
            RCLCPP_INFO(this->get_logger(),"-----Charging state-----");
            charge_state = true;
        }
    }
    else
    {
        if(charge_state == true)
        {
            RCLCPP_INFO(this->get_logger(),"-----DisCharging state-----");
            charge_state = false;
        }
    }


}

void CAN_BAT_ADL24::debugging_out()
{
    RCLCPP_INFO(this->get_logger(),"------------------------------------------- \n");
    RCLCPP_INFO(this->get_logger(),"charge_state: %s \n", charge_state==true?"ON":"OFF");

    RCLCPP_INFO(this->get_logger(),"BATTERY_charge_state: %s \n", charge_state==true?"ON":"OFF");
    RCLCPP_INFO(this->get_logger(),"bms_chargervoltage: %f \n", bms_chargervoltage);
    RCLCPP_INFO(this->get_logger(),"bms_chargercurrent: %f \n", bms_chargercurrent);
    RCLCPP_INFO(this->get_logger(),"charge_current: %f \n", charge_current);
    RCLCPP_INFO(this->get_logger(),"bms_chargerstatus: %s \n", bms_chargerstatus==true?"ON":"OFF");
    RCLCPP_INFO(this->get_logger(),"bms_chargercmd: %s \n", bms_chargercmd==true?"ON":"OFF");
    RCLCPP_INFO(this->get_logger(),"------------------------------------------- \n\n");
}

void CAN_BAT_ADL24::battery_Error_publish(int error_code)
{
    /*
  error_type:
  error_code:
  error_msg:
  source_node: "MOTOR";
  level:
   */
  if(robotError_pub_on)
  {
    switch(error_code){
        case 1:	// CAN initial Com. Error
            robotError_msg.error_type = 1001;
            robotError_msg.error_code = error_code + 99;
            robotError_msg.error_msg = "Battery: Initial CAN Com. Error";
            robotError_msg.level = "ERROR";
            RCLCPP_ERROR(this->get_logger(), "Battery: Initial CAN Com. Error");
            robotError_pub_->publish(robotError_msg);
            break;
        case 2:	// CAN alive Com. Error
            robotError_msg.error_type = 1002;
            robotError_msg.error_code = error_code + 99;
            robotError_msg.error_msg = "Battery: CAN Comm Error";
            robotError_msg.level = "ERROR";
            RCLCPP_ERROR(this->get_logger(), "Battery: CAN Comm Error");
            robotError_pub_->publish(robotError_msg);
            break;
        case 3:	// need to charge
            robotError_msg.error_type = 1003;
            robotError_msg.error_code = error_code + 99;
            robotError_msg.error_msg = "Battery: need to charge";
            robotError_msg.level = "WARN";
            RCLCPP_WARN(this->get_logger(), "Battery: need to charge");
            robotError_pub_->publish(robotError_msg);
            break;
        case 4:	// battery over Voltage
            robotError_msg.error_type = 1004;
            robotError_msg.error_code = error_code + 99;
            robotError_msg.error_msg = "Battery: Over Voltage";
            robotError_msg.level = "WARN";
            RCLCPP_WARN(this->get_logger(), "Battery: Over Voltage");
            robotError_pub_->publish(robotError_msg);
            break;
        case 5:	// battery charge over Current
            robotError_msg.error_type = 1005;
            robotError_msg.error_code = error_code + 99;
            robotError_msg.error_msg = "Battery: charge Over Current";
            robotError_msg.level = "WARN";
            RCLCPP_WARN(this->get_logger(), "Battery: charge Over Current");
            robotError_pub_->publish(robotError_msg);
            break;
        case 6:	// battery charge Over Temperature
            robotError_msg.error_type = 1006;
            robotError_msg.error_code = error_code + 99;
            robotError_msg.error_msg = "Battery: charge Over Temperature";
            robotError_msg.level = "WARN";
            RCLCPP_WARN(this->get_logger(), "Battery: charge Over Temperature");
            robotError_pub_->publish(robotError_msg);
            break;
      default:
         break;
    }
  }
}

void CAN_BAT_ADL24::check_battery_state()
{
    if(charge_exception_msg_on == true && (SOC < level_to_charge) && (SOC > 0))
    {
        // publish topic message: need to charging
        battery_Error_publish(3);
    }

    if(battery_Over_voltage)
    {
        battery_Error_publish(4);
    }

    if(battery_Charge_Over_Current)
    {
        battery_Error_publish(5);
    }

    if(battery_Charge_Over_Temp)
    {
        battery_Error_publish(6);
    }

}

void CAN_BAT_ADL24::battery_publisher()
{
//  RCLCPP_INFO(this->get_logger(),"battery_publisher(); \n");

    if(Bat_vol_pub_on)
    {
        //remained voltage;
        Voltage_msg.data = voltage;
        Bat_vol_pub_->publish(Voltage_msg);
    }

    if(Bat_charging_amp_pub_on)
    {
        //charging ampere;
        charging_Ampere_msg.data = charge_current;
        Bat_charging_amp_pub_->publish(charging_Ampere_msg);
    }

   if(Bat_discharging_amp_pub_on)
    {
        //discharging ampere;
        discharging_Ampere_msg.data = discharge_current;
        Bat_discharging_amp_pub_->publish(discharging_Ampere_msg);
    }

    if(Bat_per_pub_on)
    {
        //percentage;
        Percentage_msg.data = SOC;
        Bat_per_pub_->publish(Percentage_msg);
    }

    if(Bat_state_pub_on)
    {
        Charge_state_msg.data = charge_state;
        Bat_state_pub_->publish(Charge_state_msg);
    }

    if(bms_loadvoltage_pub_on)
    {
        // bms_loadvoltage
        bms_loadvoltage_msg.data = bms_loadvoltage;
        bms_loadvoltage_pub_->publish(bms_loadvoltage_msg);
    }

    if(bms_chargervoltage_pub_on)
    {
        // bms_chargervoltage
        bms_chargervoltage_msg.data = bms_chargervoltage;
        bms_chargervoltage_pub_->publish(bms_chargervoltage_msg);
    }

    if(bms_chargercurrent_pub_on)
    {
        // bms_chargervoltage
        bms_chargercurrent_msg.data = bms_chargercurrent;
        bms_chargercurrent_pub_->publish(bms_chargercurrent_msg);
    }
}

void CAN_BAT_ADL24::BATTERY_GetMsg1(s16 *ReceiveMSG)
{
	voltage 			= ReceiveMSG[0] * voltage_scale;
	charge_current 		= ReceiveMSG[1] * current_scale;
	discharge_current 	= ReceiveMSG[2] * current_scale;
	SOC 				= (ReceiveMSG[3] & 0xff);
	remaining_time 		= (ReceiveMSG[3] >> 8) * remaining_time_scale;
}

void CAN_BAT_ADL24::BATTERY_GetMsg2(s16 *ReceiveMSG)
{
	remaining_capacity 	= ReceiveMSG[0];
	SOH 				= (ReceiveMSG[1] & 0xff);
	BMS_ver 			= (ReceiveMSG[1] >> 8) * BMS_ver_scale;
	Full_Capacity 		= ReceiveMSG[2];
}

void CAN_BAT_ADL24::BATTERY_GetMsg3(s16 *ReceiveMSG)
{
	u16 bit;

	bit = (ReceiveMSG[0] & 0xfff);
	// bit 0: discharge, 		bit 1: charge
	// bit 2: OV(Over voltage)	bit 3: UV(Under voltage)
	// bit 4: COC(Charge Over Current)
	// bit 5: DOC(Discharge Overcurrent)
	// bit 6: DOT(Discharge Over Temp)
	// bit 7: DUT(Discharge Under Temp)
	// bit 8: N/A
	// bit 9: SC(Short circuit)
	// bit 10: COT(Charge Over Temp)
	// bit 11: CUT(Charge Under Temp)
    charge_state 					=  (bit & 0x0002)? true:false;	// bit 1
	battery_Over_voltage 			=  (bit & 0x0004)? true:false;	// bit 2
	battery_Under_voltage 			=  (bit & 0x0008)? true:false;	// bit 3

	battery_Charge_Over_Current 	=  (bit & 0x0010)? true:false;	// bit 4
	battery_Discharge_Over_Current 	=  (bit & 0x0020)? true:false;	// bit 5
	battery_Discharge_Over_Temp 	=  (bit & 0x0040)? true:false;	// bit 6
	battery_Discharge_Under_Temp 	=  (bit & 0x0080)? true:false;	// bit 7

	//N/A 							=  (bit & 0x0100)? true:false;	// bit 8
	//SC(Short circuit) 			=  (bit & 0x0200)? true:false;	// bit 9
	battery_Charge_Over_Temp 		=  (bit & 0x0400)? true:false;	// bit 10
	battery_Charge_Under_Temp 		=  (bit & 0x0800)? true:false;	// bit 11
}

void CAN_BAT_ADL24::BMS_GetMsg(s16 *ReceiveMSG)
{
	u8 bit;

	bms_loadvoltage 		= ReceiveMSG[0] * 0.01;
	bms_chargervoltage 		= ReceiveMSG[1] * 0.01;

	bit = (ReceiveMSG[2] & 0xff);

	if(bit & 0x01)
		bms_chargerstatus = true;
	else
		bms_chargerstatus = false;

	bms_chargercurrent 		= ReceiveMSG[3] * 0.01 ;

}
