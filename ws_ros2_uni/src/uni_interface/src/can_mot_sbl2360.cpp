#include "uni_interface/can_mot_sbl2360.h"
#include "uni_interface/modbusMap.h"

using namespace std::chrono_literals;
using namespace std::placeholders;

// ******** constructors ******** //
CAN_MOT_SBL2360::CAN_MOT_SBL2360():
    Node("can_mot_node")
{

#ifdef _PUB_TF
    RCLCPP_INFO(this->get_logger(),"pub_odom_tf: " << "true");
#endif

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    this->declare_parameter("base_frame");
    this->declare_parameter("cmdvel_topic");
    this->declare_parameter("cmd_vel_quesize");
    this->declare_parameter("odom_pub_on");
    this->declare_parameter("odom_topic");
    this->declare_parameter("odom_frame");
    this->declare_parameter("open_loop");
    this->declare_parameter("wheel_radius");
    this->declare_parameter("track_width");
    this->declare_parameter("gear_ratio");
    this->declare_parameter("encoder_ppr");
    this->declare_parameter("encoder_cpr");
    this->declare_parameter("torque_constant");
    this->declare_parameter("numof_pole_pairs");
    this->declare_parameter("operating_mode");
    this->declare_parameter("swap_winding");
    this->declare_parameter("mot_amps_limit");
    this->declare_parameter("max_speed_command");
    this->declare_parameter("max_acceleration_rate");
    this->declare_parameter("max_deceleration_rate");
    this->declare_parameter("fault_deceleration_rate");
    this->declare_parameter("mot_direction_R");
    this->declare_parameter("mot_direction_L");
    this->declare_parameter("pwm_frequency");
    this->declare_parameter("auto_sending_rate_ms");
    this->declare_parameter("rightwheel_gain_p");
    this->declare_parameter("rightwheel_gain_i");
    this->declare_parameter("rightwheel_gain_d");
    this->declare_parameter("leftwheel_gain_p");
    this->declare_parameter("leftwheel_gain_i");
    this->declare_parameter("leftwheel_gain_d");
    this->declare_parameter("encoder_pub_on");
    this->declare_parameter("encoder_pub_topic");
    this->declare_parameter("current_pub_on");
    this->declare_parameter("current_pub_topic");
    this->declare_parameter("power_pub_on");
    this->declare_parameter("power_pub_topic");
    this->declare_parameter("robotError_pub_on");
    this->declare_parameter("robotError_pub_topic");
    this->declare_parameter("emgbutton_sub_on");
    this->declare_parameter("emgbutton_sub_topic");
    this->declare_parameter("encoder_count_relative");

    this->get_parameter_or<std::string>("base_frame", base_frame, std::string("base_link"));
    RCLCPP_INFO(this->get_logger(),"base_frame: %s", base_frame.c_str());

    this->get_parameter_or<std::string>("cmdvel_topic", cmdvel_topic, std::string("/cmd_vel"));
    RCLCPP_INFO(this->get_logger(),"cmdvel_topic: %s", cmdvel_topic.c_str());

    this->get_parameter_or<int>("cmd_vel_quesize", cmd_vel_quesize, 1);
    RCLCPP_INFO(this->get_logger(),"cmd_vel_quesize: %d", cmd_vel_quesize);

    this->get_parameter_or<bool>("odom_pub_on", odom_pub_on, true);
    RCLCPP_INFO(this->get_logger(),"odom_pub_on: %d", odom_pub_on);

    this->get_parameter_or<std::string>("odom_topic", odom_topic, std::string("odom"));
    RCLCPP_INFO(this->get_logger(),"odom_topic: %s", odom_topic.c_str());

    this->get_parameter_or<std::string>("odom_frame", odom_frame, std::string("odom"));
    RCLCPP_INFO(this->get_logger(),"odom_frame: %s", odom_frame.c_str());

    this->get_parameter_or<bool>("open_loop", open_loop, LOOP_MODE);
    RCLCPP_INFO(this->get_logger(),"open_loop: %d", open_loop);

    // wheel radius[m]: r, wheel circumference: 2*pi*r
    this->get_parameter_or<double>("wheel_radius", wheel_radius, ROBOT_WHEEL_RADIUS);
    RCLCPP_INFO(this->get_logger(),"wheel_radius: %lf", wheel_radius);

    wheel_circumference = 2*pi*wheel_radius;
    RCLCPP_INFO(this->get_logger(),"wheel_circumference: %lf", wheel_circumference);

    // distance between the centers of the two wheels[m]
    this->get_parameter_or<double>("track_width", track_width, ROBOT_BASE);
    RCLCPP_INFO(this->get_logger(),"track_width: %lf", track_width);

    // gear ratio
    this->get_parameter_or<double>("gear_ratio", gear_ratio, ROBOT_GEAR_RATIO);
    RCLCPP_INFO(this->get_logger(),"gear_ratio: %lf", gear_ratio);

    // encoder ppr
    this->get_parameter_or<int>("encoder_ppr", encoder_ppr, ROBOT_ENCODER_PPR);
    RCLCPP_INFO(this->get_logger(),"encoder_ppr: %d", encoder_ppr);

    // encoder cpr
    this->get_parameter_or<int>("encoder_cpr", encoder_cpr, ROBOT_ENCODER_CPR);
    RCLCPP_INFO(this->get_logger(),"encoder_cpr: %d", encoder_cpr);

    // torque_constant
    //this->get_parameter_or<int>("torque_constant", torque_constant, 50);
    this->get_parameter_or<int>("torque_constant", torque_constant, 430);
    RCLCPP_INFO(this->get_logger(),"torque_constant: %d", torque_constant);

    // numof_pole_pairs
    //this->get_parameter_or<int>("numof_pole_pairs", numof_pole_pairs, 3);
    this->get_parameter_or<int>("numof_pole_pairs", numof_pole_pairs, 10);
    RCLCPP_INFO(this->get_logger(),"numof_pole_pairs: %d", numof_pole_pairs);

    // operating_mode
    //this->get_parameter_or<int>("operating_mode", operating_mode, 0);
    this->get_parameter_or<int>("operating_mode", operating_mode, 1);
    RCLCPP_INFO(this->get_logger(),"operating_mode: %d", operating_mode);

    // swap_winding
    this->get_parameter_or<int>("swap_winding", swap_winding, 0);
    RCLCPP_INFO(this->get_logger(),"swap_winding: %d", swap_winding);

    // Amp Limit
    //this->get_parameter_or<int>("mot_amps_limit", mot_amps_limit, 94);
    this->get_parameter_or<int>("mot_amps_limit", mot_amps_limit, 75);
    RCLCPP_INFO(this->get_logger(),"mot_amps_limit: %d", mot_amps_limit);

    // Max RPM
    //this->get_parameter_or<int>("max_speed_command", max_speed_command, 3500);
    this->get_parameter_or<int>("max_speed_command", max_speed_command, 250);
    RCLCPP_INFO(this->get_logger(),"max_speed_command: %d", max_speed_command);

    // Max acceleration
    //this->get_parameter_or<int>("max_acceleration_rate", max_acceleration_rate, 5000);
    this->get_parameter_or<int>("max_acceleration_rate", max_acceleration_rate, 500);
    RCLCPP_INFO(this->get_logger(),"max_acceleration_rate: %d", max_acceleration_rate);

    // Max deceleration
    //this->get_parameter_or<int>("max_deceleration_rate", max_deceleration_rate, 7000);
    this->get_parameter_or<int>("max_deceleration_rate", max_deceleration_rate, 700);
    RCLCPP_INFO(this->get_logger(),"max_deceleration_rate: %d", max_deceleration_rate);

    // Fault deceleration
    this->get_parameter_or<int>("fault_deceleration_rate", fault_deceleration_rate, 20000);
    RCLCPP_INFO(this->get_logger(),"fault_deceleration_rate: %d", fault_deceleration_rate);

    // Set Motor Direction
    this->get_parameter_or<int>("mot_direction_R", mot_direction_R, 0);
    this->get_parameter_or<int>("mot_direction_L", mot_direction_L, 1);
    RCLCPP_INFO(this->get_logger(),"mot_direction_R: %d, mot_direction_R: %d", mot_direction_R, mot_direction_L);

    // Set PWM frequency
    //this->get_parameter_or<int>("pwm_frequency", pwm_frequency, 250);
    this->get_parameter_or<int>("pwm_frequency", pwm_frequency, 160);
    RCLCPP_INFO(this->get_logger(),"pwm_frequency: %d", pwm_frequency);

    // Set automatic sending rate
    this->get_parameter_or<int>("auto_sending_rate_ms", auto_sending_rate_ms, 10);
    RCLCPP_INFO(this->get_logger(),"auto_sending_rate_ms: %d", auto_sending_rate_ms);

    // if(auto_sending_rate_ms == 4)   // 125.0 Hz
    //      auto_sending_rate_ms_cnt = 8;
    // else if(auto_sending_rate_ms == 3)   // 166.0 Hz
    //      auto_sending_rate_ms_cnt = 6;
    // else if(auto_sending_rate_ms == 2)   // 250.0 Hz
    //      auto_sending_rate_ms_cnt = 4;
    // else                                //100.0 Hz
    //     auto_sending_rate_ms_cnt = 10;


    // PID Gain of wheels
    // this->get_parameter_or<int>("rightwheel_gain_p", rightwheel_gain_p, 0);
    // this->get_parameter_or<int>("rightwheel_gain_i", rightwheel_gain_i, 10);
    this->get_parameter_or<int>("rightwheel_gain_p", rightwheel_gain_p, 2);
    this->get_parameter_or<int>("rightwheel_gain_i", rightwheel_gain_i, 90);
    this->get_parameter_or<int>("rightwheel_gain_d", rightwheel_gain_d, 0);
    RCLCPP_INFO(this->get_logger(),"rightwheel_gain p:%d, i:%d, d:%d", rightwheel_gain_p, rightwheel_gain_i, rightwheel_gain_d);

    // this->get_parameter_or<int>("leftwheel_gain_p" , leftwheel_gain_p, 0);
    // this->get_parameter_or<int>("leftwheel_gain_i" , leftwheel_gain_i, 10);
    this->get_parameter_or<int>("leftwheel_gain_p" , leftwheel_gain_p, 2);
    this->get_parameter_or<int>("leftwheel_gain_i" , leftwheel_gain_i, 90);
    this->get_parameter_or<int>("leftwheel_gain_d" , leftwheel_gain_d, 0);
    RCLCPP_INFO(this->get_logger(),"leftwheel_gain p:%d, i:%d, d:%d", leftwheel_gain_p, leftwheel_gain_i, leftwheel_gain_d);

    this->get_parameter_or<bool>("encoder_pub_on", encoder_pub_on, true);
    RCLCPP_INFO(this->get_logger(),"encoder_pub_on: %d", encoder_pub_on);
    this->get_parameter_or<std::string>("encoder_pub_topic", encoder_pub_topic, std::string("robot/motor/encoder"));
    RCLCPP_INFO(this->get_logger(),"encoder_pub_topic: %s", encoder_pub_topic.c_str());

    this->get_parameter_or<bool>("current_pub_on", current_pub_on, true);
    RCLCPP_INFO(this->get_logger(),"current_pub_on: %d", current_pub_on);
    this->get_parameter_or<std::string>("current_pub_topic", current_pub_topic, std::string("robot/motor/ampere"));
    RCLCPP_INFO(this->get_logger(),"current_pub_topic: %s", current_pub_topic.c_str());

    this->get_parameter_or<bool>("power_pub_on", power_pub_on, true);
    RCLCPP_INFO(this->get_logger(),"power_pub_on: %d", power_pub_on);
    this->get_parameter_or<std::string>("power_pub_topic", power_pub_topic, std::string("robot/motor/power"));
    RCLCPP_INFO(this->get_logger(),"power_pub_topic: %s", power_pub_topic.c_str());

    this->get_parameter_or<bool>("robotError_pub_on", robotError_pub_on, true);
    RCLCPP_INFO(this->get_logger(),"robotError_pub_on: %d", robotError_pub_on);
    this->get_parameter_or<std::string>("robotError_pub_topic", robotError_pub_topic, std::string("robot/ErrorMsg"));
    RCLCPP_INFO(this->get_logger(),"robotError_pub_topic: %s", robotError_pub_topic.c_str());

    this->get_parameter_or<bool>("emgbutton_sub_on", emgbutton_sub_on, true);
    RCLCPP_INFO(this->get_logger(),"emgbutton_sub_on: %d", emgbutton_sub_on);
    this->get_parameter_or<std::string>("emgbutton_sub_topic", emgbutton_sub_topic, std::string("/robot/button/emergency_state"));
    RCLCPP_INFO(this->get_logger(),"emgbutton_sub_topic: %s", emgbutton_sub_topic.c_str());

    this->get_parameter_or<bool>("encoder_count_relative", encoder_count_relative, true);
    RCLCPP_INFO(this->get_logger(),"encoder_count_relative: %d", encoder_count_relative);

    if(encoder_count_relative)
        RCLCPP_INFO(this->get_logger(), "encoder count : relative");
    else
        RCLCPP_INFO(this->get_logger(), "encoder count : absolute");

    set_encoder_right_value = 0;
    set_encoder_left_value = 0;

    faultflags = 0xffff;
    runtimestatusflags_right = 0xffff;
    runtimestatusflags_left = 0xffff;
    statusflags = 0xffff;
    bMicroBasicscriptrunning = true;

    start();
}

void CAN_MOT_SBL2360::msg_publisher()
{
    MOT_GetData0((s16 *)&DataMem[MOT_CAN_Rx050_0]);
    MOT_GetData1((s16 *)&DataMem[MOT_CAN_Rx051_0]);
    MOT_GetData2((s16 *)&DataMem[MOT_CAN_Rx052_0]);
    MOT_GetData3((s16 *)&DataMem[MOT_CAN_Rx053_0]);
}

CAN_MOT_SBL2360::~CAN_MOT_SBL2360()
{
    RCLCPP_WARN(this->get_logger(), "Exiting CAN_MOT_SBL2360");
}

uint32_t CAN_MOT_SBL2360::millis()
{
    rclcpp::Time now = this->now();
    return (uint32_t)(now.nanoseconds()/1000000);
}

void CAN_MOT_SBL2360::MOT_GetData0(s16 *ReceiveMSG)
{
    odom_encoder_right_prev   = odom_encoder_right;
    odom_encoder_left_prev    = odom_encoder_left;

	odom_encoder_right 		= ((ReceiveMSG[1] <<16) | ReceiveMSG[0]);
    odom_encoder_left 		= ((ReceiveMSG[3] <<16) | ReceiveMSG[2]);

    motor_encoder_publish();
}


void CAN_MOT_SBL2360::MOT_GetData1(s16 *ReceiveMSG)
{
    Amp_right 		    = ReceiveMSG[0];
	Amp_left 		    = ReceiveMSG[1];
    right_power 		= ReceiveMSG[2];
	left_power 		    = ReceiveMSG[3];

    motor_current_publish();
    motor_power_publish();
}


void CAN_MOT_SBL2360::MOT_GetData2(s16 *ReceiveMSG)
{
    ReceiveMSG[0] = ReceiveMSG[0];
}


void CAN_MOT_SBL2360::Checkfaultflags(uint16_t faultflagsdata)
/* faultflags */
// bit0 = Overheat,                         bit1 = Overvoltage,                 bit2 = Undervoltage,                bit3 = Short circuit
// bit4 = Emergency stop,                   bit5 = Motor/Sensor Setup fault,    bit6 = MOSFET failure,              bit7 = Default configuration loaded at startup
{
    if(faultflagsdata == 0)
    {
        if(faultflags != 0)
        {
            faultflags = faultflagsdata;
            //motor_Error_publish(19);    //clear fault flag
            return;
        }
        return;
    }
    else
       faultflags = faultflagsdata;

    //bOverheat 					    =  ((faultflags & 0x0001)== 0x0001)?true:false;	// bit 0
    if((faultflags & 0x0001) )  // bit 0
    {
        if(bOverheat == false)
        {
            bOverheat  =  true;
            //motor_Error_publish(10);
        }
    }
    else
        bOverheat  =  false;

    //bOvervoltage 				    =  ((faultflags & 0x0002)== 0x0002)?true:false;	// bit 1
    if((faultflags & 0x0002))  // bit 1
    {
        if(bOvervoltage == false)
        {
            bOvervoltage  =  true;
            //motor_Error_publish(11);
        }
    }
    else
        bOvervoltage  =  false;

    //bUndervoltage 			        =  ((faultflags & 0x0004)== 0x0004)?true:false;	// bit 2
    if((faultflags & 0x0004))  // bit 2
    {
        if(bUndervoltage == false)
        {
            bUndervoltage  =  true;
            //motor_Error_publish(12);
        }
    }
    else
        bUndervoltage  =  false;

    //bShortcircuit 			        =  ((faultflags & 0x0008)== 0x0008)?true:false;	// bit 3
    if((faultflags & 0x0008))  // bit 3
    {
        if(bShortcircuit == false)
        {
            bShortcircuit  =  true;
            //motor_Error_publish(13);
        }
    }
    else
        bShortcircuit  =  false;

    //bEmergencystop 	                =  ((faultflags & 0x0010)== 0x0010)?true:false;	// bit 4
    if((faultflags & 0x0010))  // bit 4
    {
        if(bEmergencystop == false)
        {
            bEmergencystop  =  true;
            //motor_Error_publish(14);
        }
    }
    else
        bEmergencystop  =  false;

    //bSetupfault 	                =  ((faultflags & 0x0020)== 0x0020)?true:false;	// bit 5
    if((faultflags & 0x0020))  // bit 5
    {
        if(bSetupfault == false)
        {
            bSetupfault  =  true;
            //motor_Error_publish(15);
        }
    }
    else
        bSetupfault  =  false;

    //bMOSFETfailure 	                =  ((faultflags & 0x0040)== 0x0040)?true:false;	// bit 6
    if((faultflags & 0x0040))  // bit 6
    {
        if(bMOSFETfailure == false)
        {
            bMOSFETfailure  =  true;
            //motor_Error_publish(16);
        }
    }
    else
        bMOSFETfailure  =  false;

    //bDefaultconfigurationloaded     =  ((faultflags & 0x0080)== 0x0080)?true:false;	// bit 7
    if((faultflags & 0x0080))  // bit 7
    {
        if(bDefaultconfigurationloaded == false)
        {
            bDefaultconfigurationloaded  =  true;
            //motor_Error_publish(17);
        }
    }
    else
        bDefaultconfigurationloaded  =  false;
}

void CAN_MOT_SBL2360::Checkruntimestatus_right(uint16_t runtimestatusflags_rightdata)
/* runtimestatusflags_right */
// bit0 = Amps Limit currently active,      bit1 = Motor stalled,               bit2 = Loop Error detected,         bit3 = Safety Stop active
// bit4 = Forward Limit triggered,          bit5 = Reverse Limit triggered,     bit6 = Amps Trigger activated
{
    if(runtimestatusflags_rightdata == 0)
    {
        if(runtimestatusflags_right != 0)
        {
            runtimestatusflags_right = runtimestatusflags_rightdata;
            //motor_Error_publish(29);    //clear runtimestatusflags_right
            return;
        }
        return;
    }
    else
       runtimestatusflags_right = runtimestatusflags_rightdata;

    //bAmpsLimitactive_right 			=  ((runtimestatusflags_right & 0x0001)== 0x0001)?true:false;	// bit 0
    if((runtimestatusflags_right & 0x0001))  // bit 0
    {
        if(bAmpsLimitactive_right == false)
        {
            bAmpsLimitactive_right  =  true;
            //motor_Error_publish(20);
        }
    }
    else
        bAmpsLimitactive_right  =  false;

    //bMotorstalled_right 			=  ((runtimestatusflags_right & 0x0002)== 0x0002)?true:false;	// bit 1
    if((runtimestatusflags_right & 0x0002))  // bit 1
    {
        if(bMotorstalled_right == false)
        {
            bMotorstalled_right  =  true;
            //motor_Error_publish(21);
        }
    }
    else
        bMotorstalled_right  =  false;

	//bLoopError_right 			    =  ((runtimestatusflags_right & 0x0004)== 0x0004)?true:false;	// bit 2
    if((runtimestatusflags_right & 0x0004))  // bit 2
    {
        if(bLoopError_right == false)
        {
            bLoopError_right  =  true;
            //motor_Error_publish(22);
        }
    }
    else
        bLoopError_right  =  false;

	//bSafetyStop_right 			    =  ((runtimestatusflags_right & 0x0008)== 0x0008)?true:false;	// bit 3
    if((runtimestatusflags_right & 0x0008))  // bit 3
    {
        if(bSafetyStop_right == false)
        {
            bSafetyStop_right  =  true;
            //motor_Error_publish(23);
        }
    }
    else
        bSafetyStop_right  =  false;

    //bForwardLimit_right 	        =  ((runtimestatusflags_right & 0x0010)== 0x0010)?true:false;	// bit 4
    if((runtimestatusflags_right & 0x0010))  // bit 4
    {
        if(bForwardLimit_right == false)
        {
            bForwardLimit_right  =  true;
            //motor_Error_publish(24);
        }
    }
    else
        bForwardLimit_right  =  false;

	//bReverseLimit_right 	        =  ((runtimestatusflags_right & 0x0020)== 0x0020)?true:false;	// bit 5
    if((runtimestatusflags_right & 0x0020))  // bit 5
    {
        if(bReverseLimit_right == false)
        {
            bReverseLimit_right  =  true;
            //motor_Error_publish(25);
        }
    }
    else
        bReverseLimit_right  =  false;

	//bAmpsTrigger_right 	            =  ((runtimestatusflags_right & 0x0040)== 0x0040)?true:false;	// bit 6
    if((runtimestatusflags_right & 0x0040))  // bit 6
    {
        if(bAmpsTrigger_right == false)
        {
            bAmpsTrigger_right  =  true;
            //motor_Error_publish(26);
        }
    }
    else
        bAmpsTrigger_right  =  false;

}

void CAN_MOT_SBL2360::Checkruntimestatus_left(uint16_t runtimestatusflags_leftdata)
/* runtimestatusflags_left */
// bit0 = Amps Limit currently active,      bit1 = Motor stalled,               bit2 = Loop Error detected,         bit3 = Safety Stop active
// bit4 = Forward Limit triggered,          bit5 = Reverse Limit triggered,     bit6 = Amps Trigger activated
{
    if(runtimestatusflags_leftdata == 0)
    {
        if(runtimestatusflags_left != 0)
        {
            runtimestatusflags_left = runtimestatusflags_leftdata;
            //motor_Error_publish(39);    //clear runtimestatusflags_left
            return;
        }
        return;
    }
    else
       runtimestatusflags_left = runtimestatusflags_leftdata;

    //bAmpsLimitactive_left 			=  ((runtimestatusflags_left & 0x0001)== 0x0001)?true:false;	// bit 0
    if((runtimestatusflags_left & 0x0001))  // bit 0
    {
        if(bAmpsLimitactive_left == false)
        {
            bAmpsLimitactive_left  =  true;
            //motor_Error_publish(30);
        }
    }
    else
        bAmpsLimitactive_left  =  false;

    //bMotorstalled_left 			    =  ((runtimestatusflags_left & 0x0002)== 0x0002)?true:false;	// bit 1
    if((runtimestatusflags_left & 0x0002))  // bit 1
    {
        if(bMotorstalled_left == false)
        {
            bMotorstalled_left  =  true;
            //motor_Error_publish(31);
        }
    }
    else
        bMotorstalled_left  =  false;

	//bLoopError_left 			    =  ((runtimestatusflags_left & 0x0004)== 0x0004)?true:false;	// bit 2
    if((runtimestatusflags_left & 0x0004))  // bit 2
    {
        if(bLoopError_left == false)
        {
            bLoopError_left  =  true;
            //motor_Error_publish(32);
        }
    }
    else
        bLoopError_left  =  false;

	//bSafetyStop_left 			    =  ((runtimestatusflags_left & 0x0008)== 0x0008)?true:false;	// bit 3
    if((runtimestatusflags_left & 0x0008))  // bit 3
    {
        if(bSafetyStop_left == false)
        {
            bSafetyStop_left  =  true;
            //motor_Error_publish(33);
        }
    }
    else
        bSafetyStop_left  =  false;

    //bForwardLimit_left 	            =  ((runtimestatusflags_left & 0x0010)== 0x0010)?true:false;	// bit 4
    if((runtimestatusflags_left & 0x0010))  // bit 4
    {
        if(bForwardLimit_left == false)
        {
            bForwardLimit_left  =  true;
            //motor_Error_publish(34);
        }
    }
    else
        bForwardLimit_left  =  false;

	//bReverseLimit_left 	            =  ((runtimestatusflags_left & 0x0020)== 0x0020)?true:false;	// bit 5
    if((runtimestatusflags_left & 0x0020))  // bit 5
    {
        if(bReverseLimit_left == false)
        {
            bReverseLimit_left  =  true;
            //motor_Error_publish(35);
        }
    }
    else
        bReverseLimit_left  =  false;

	//bAmpsTrigger_left 	            =  ((runtimestatusflags_left & 0x0040)== 0x0040)?true:false;	// bit 6
    if((runtimestatusflags_left & 0x0040))  // bit 6
    {
        if(bAmpsTrigger_left == false)
        {
            bAmpsTrigger_left  =  true;
            //motor_Error_publish(36);
        }
    }
    else
        bAmpsTrigger_left  =  false;

}

void CAN_MOT_SBL2360::Checkstatusflags(uint16_t statusflagsdata)
/* statusflags */
// bit0 = Serial mode,                      bit1 =  Pulse mode ,                bit2 =  Analog mode,                bit3 = Power stage off
// bit4 = Stall detected ,                  bit5 =  At limit,                   bit6 =  Unused ,                    bit7 =  MicroBasic script running
// bit8 = Motor/Sensor Tuning mode
{
    if(statusflagsdata == 0x0080)
    {
        if(statusflags != 0x0080)
        {
            statusflags = statusflagsdata;
            //motor_Error_publish(49);    //clear statusflags
            return;
        }
        return;
    }
    else
       statusflags = statusflagsdata;

    //bSerialMode 					=  ((statusflags & 0x0001)== 0x0001)?true:false;	// bit 0
    if((statusflags & 0x0001) )  // bit 0
    {
        if(bSerialMode == false)
        {
            bSerialMode  =  true;
            //motor_Error_publish(40);
        }
    }
    else
    {
        bSerialMode  =  false;
    }


    //bPulsemode 				        =  ((statusflags & 0x0002)== 0x0002)?true:false;	// bit 1
    if((statusflags & 0x0002))  // bit 1
    {
        if(bPulsemode == false)
        {
            bPulsemode  =  true;
            //motor_Error_publish(41);
        }
    }
    else
        bPulsemode  =  false;

	//bAnalogmode 			        =  ((statusflags & 0x0004)== 0x0004)?true:false;	// bit 2
    if((statusflags & 0x0004))  // bit 2
    {
        if(bAnalogmode == false)
        {
            bAnalogmode  =  true;
            //motor_Error_publish(42);
        }
    }
    else
        bAnalogmode  =  false;

	//bPowerstageoff 			        =  ((statusflags & 0x0008)== 0x0008)?true:false;	// bit 3
    if((statusflags & 0x0008))  // bit 3
    {
        if(bPowerstageoff == false)
        {
            bPowerstageoff  =  true;
            //motor_Error_publish(43);
        }
    }
    else
        bPowerstageoff  =  false;

    //bStalldetected 	                =  ((statusflags & 0x0010)== 0x0010)?true:false;	// bit 4
    if((statusflags & 0x0010))  // bit 4
    {
        if(bStalldetected == false)
        {
            bStalldetected  =  true;
            //motor_Error_publish(44);
        }
    }
    else
        bStalldetected  =  false;

	//bAtlimit 	                    =  ((statusflags & 0x0020)== 0x0020)?true:false;	// bit 5
    if((statusflags & 0x0020))  // bit 5
    {
        if(bAtlimit == false)
        {
            bAtlimit  =  true;
            //motor_Error_publish(45);
        }
    }
    else
        bAtlimit  =  false;

	//bUnused 	                    =  ((statusflags & 0x0040)== 0x0040)?true:false;	// bit 6
    if((statusflags & 0x0040))  // bit 6
    {
        if(bUnused == false)
        {
            bUnused  =  true;
            //motor_Error_publish(46);
        }
    }
    else
        bUnused  =  false;

	//bMicroBasicscriptrunning        =  ((statusflags & 0x0080)== 0x0080)?true:false;	// bit 7
    if((statusflags & 0x0080))  // bit 7
    {
        bMicroBasicscriptrunning  =  true;
    }
    else
    {
        if(bMicroBasicscriptrunning == true)
        {
            bMicroBasicscriptrunning  =  false;
            //motor_Error_publish(47);
        }
    }


    //bMotorSensorTunningmode        =  ((statusflags & 0x0100)== 0x0080)?true:false;	// bit 8
    if((statusflags & 0x0100) )  // bit 7
    {
        if(bMotorSensorTunningmode == false)
        {
            bMotorSensorTunningmode  =  true;
            //motor_Error_publish(48);
        }
    }
    else
        bMotorSensorTunningmode  =  false;

}

void CAN_MOT_SBL2360::MOT_GetData3(s16 *ReceiveMSG)
{
	uint16_t faultflagsdata 		         = (uint16_t)ReceiveMSG[0];
	uint16_t runtimestatusflags_rightdata 	 = (uint16_t)ReceiveMSG[1];
	uint16_t runtimestatusflags_leftdata 	 = (uint16_t)ReceiveMSG[2];
	uint16_t statusflagsdata 		         = (uint16_t)ReceiveMSG[3];

    Checkfaultflags(faultflagsdata);
    Checkruntimestatus_right(runtimestatusflags_rightdata);
    Checkruntimestatus_left(runtimestatusflags_leftdata);
    Checkstatusflags(statusflagsdata);
}

void CAN_MOT_SBL2360::statusoutdebugging()
{
    RCLCPP_DEBUG(this->get_logger(),"------------------------------------------- \n");
    RCLCPP_DEBUG(this->get_logger(),"bOverheat: %s \n", bOverheat==true?"ON":"OFF");
    RCLCPP_DEBUG(this->get_logger(),"bOvervoltage: %s \n", bOvervoltage==true?"ON":"OFF");
    RCLCPP_DEBUG(this->get_logger(),"bUndervoltage: %s \n", bUndervoltage==true?"ON":"OFF");
    RCLCPP_DEBUG(this->get_logger(),"bShortcircuit: %s \n", bShortcircuit==true?"ON":"OFF");
    RCLCPP_DEBUG(this->get_logger(),"bEmergencystop: %s \n", bEmergencystop==true?"ON":"OFF");
    RCLCPP_DEBUG(this->get_logger(),"bSetupfault: %s \n", bSetupfault==true?"ON":"OFF");
    RCLCPP_DEBUG(this->get_logger(),"bMOSFETfailure: %s \n", bMOSFETfailure==true?"ON":"OFF");
    RCLCPP_DEBUG(this->get_logger(),"bDefaultconfigurationloaded: %s \n", bDefaultconfigurationloaded==true?"ON":"OFF");

    RCLCPP_DEBUG(this->get_logger(),"bAmpsLimitactive_right: %s \n", bAmpsLimitactive_right==true?"ON":"OFF");
    RCLCPP_DEBUG(this->get_logger(),"bMotorstalled_right: %s \n", bMotorstalled_right==true?"ON":"OFF");
    RCLCPP_DEBUG(this->get_logger(),"bLoopError_right: %s \n", bLoopError_right==true?"ON":"OFF");
    RCLCPP_DEBUG(this->get_logger(),"bSafetyStop_right: %s \n", bSafetyStop_right==true?"ON":"OFF");
    RCLCPP_DEBUG(this->get_logger(),"bForwardLimit_right: %s \n", bForwardLimit_right==true?"ON":"OFF");
    RCLCPP_DEBUG(this->get_logger(),"bReverseLimit_right: %s \n", bReverseLimit_right==true?"ON":"OFF");
    RCLCPP_DEBUG(this->get_logger(),"bAmpsTrigger_right: %s \n", bAmpsTrigger_right==true?"ON":"OFF");

    RCLCPP_DEBUG(this->get_logger(),"bAmpsLimitactive_left: %s \n", bAmpsLimitactive_left==true?"ON":"OFF");
    RCLCPP_DEBUG(this->get_logger(),"bMotorstalled_left: %s \n", bMotorstalled_left==true?"ON":"OFF");
    RCLCPP_DEBUG(this->get_logger(),"bLoopError_left: %s \n", bLoopError_left==true?"ON":"OFF");
    RCLCPP_DEBUG(this->get_logger(),"bSafetyStop_left: %s \n", bSafetyStop_left==true?"ON":"OFF");
    RCLCPP_DEBUG(this->get_logger(),"bForwardLimit_left: %s \n", bForwardLimit_left==true?"ON":"OFF");
    RCLCPP_DEBUG(this->get_logger(),"bReverseLimit_left: %s \n", bReverseLimit_left==true?"ON":"OFF");
    RCLCPP_DEBUG(this->get_logger(),"bAmpsTrigger_left: %s \n", bAmpsTrigger_left==true?"ON":"OFF");

    RCLCPP_DEBUG(this->get_logger(),"bSerialMode: %s \n", bSerialMode==true?"ON":"OFF");
    RCLCPP_DEBUG(this->get_logger(),"bPulsemode: %s \n", bPulsemode==true?"ON":"OFF");
    RCLCPP_DEBUG(this->get_logger(),"bAnalogmode: %s \n", bAnalogmode==true?"ON":"OFF");
    RCLCPP_DEBUG(this->get_logger(),"bSafetyStop_left: %s \n", bSafetyStop_left==true?"ON":"OFF");

    RCLCPP_DEBUG(this->get_logger(),"bStalldetected: %s \n", bStalldetected==true?"ON":"OFF");
    RCLCPP_DEBUG(this->get_logger(),"bAtlimit: %s \n", bAtlimit==true?"ON":"OFF");
    RCLCPP_DEBUG(this->get_logger(),"bUnused: %s \n", bUnused==true?"ON":"OFF");
    RCLCPP_DEBUG(this->get_logger(),"bMicroBasicscriptrunning: %s \n", bMicroBasicscriptrunning==true?"ON":"OFF");

    RCLCPP_DEBUG(this->get_logger(),"------------------------------------------- \n\n");
}

void CAN_MOT_SBL2360::motorbreakoFF()
{
    // break off data
    /*
    controller.write("!D1 1\r");
    controller.write("!D1 2\r");
    controller.flush();
    RCLCPP_INFO(this->get_logger(),"motorbreakoFF");
    */
}

void CAN_MOT_SBL2360::motorbreakoN()
{
    /*controller.write("!G 1 0\r");
    controller.write("!G 2 0\r");
    controller.write("!S 1 0\r");
    controller.write("!S 2 0\r");

    controller.write("!D0 1\r");
    controller.write("!D0 2\r");
    controller.flush(); */
    RCLCPP_INFO(this->get_logger(),"motorbreakoN");

}

void CAN_MOT_SBL2360::EmgStop()
{
    int ret = 0;

    if(Motor_Emergencystop_cmd == false)
    {
        RCLCPP_INFO(this->get_logger(),"EmgStop cmd");
        // check sequence
        Motor_Emergencystop_cmd = true;

        SetDataMem(MOT_CAN_Tx052_0+0, 0x0001);
        SetDataMem(MOT_CAN_Tx052_0+1, 0);
        SetDataMem(MOT_CAN_Tx052_0+2, 0);
        SetDataMem(MOT_CAN_Tx052_0+3, 0);

        motor_Error_publish(3);

        if(ret != 0)
            motor_Error_publish(5);
    }

}

void CAN_MOT_SBL2360::EmgRelease()
{
    int ret = 0;

    if(Motor_Emergencystop_cmd == true)
    {
        RCLCPP_INFO(this->get_logger(),"EmgRelease cmd");
        SafetyStop();
        // check sequence
        Motor_Emergencystop_cmd = false;

        SetDataMem(MOT_CAN_Tx052_0+0, 0x0100);
        SetDataMem(MOT_CAN_Tx052_0+1, 0);
        SetDataMem(MOT_CAN_Tx052_0+2, 0);
        SetDataMem(MOT_CAN_Tx052_0+3, 0);

        motor_Error_publish(4);

        if(ret != 0)
            motor_Error_publish(5);
    }
}

void CAN_MOT_SBL2360::SafetyStop()
{
    int ret = 0;

    SetDataMem(MOT_CAN_Tx052_0+0, 0);
    SetDataMem(MOT_CAN_Tx052_0+1, 0x0001);
    SetDataMem(MOT_CAN_Tx052_0+2, 0);
    SetDataMem(MOT_CAN_Tx052_0+3, 0);

    motor_Error_publish(6);

    if(ret != 0)
        motor_Error_publish(5);
}

// cmd_vel subscriber
void CAN_MOT_SBL2360::cmdvel_callback( const geometry_msgs::msg::Twist::SharedPtr twist_msg)
{
    if(Motor_Emergencystop_cmd == false)
    {
        // wheel speed (m/s)
        float right_speed = (twist_msg->linear.x + track_width * twist_msg->angular.z / 2.0) * gear_ratio;
        float left_speed = (twist_msg->linear.x - track_width * twist_msg->angular.z / 2.0)  * gear_ratio;

        if (open_loop)
        {
            // motor power (scale 0-1000)
            right_power = right_speed / wheel_circumference * 60.0 / 82.0 * 1000.0;
            left_power = left_speed / wheel_circumference * 60.0 / 82.0 * 1000.0;
        }
        else
        {
            // motor speed (rpm)
            // V= Wr = 2*pi*r/T
            right_rpm = right_speed / wheel_circumference * 60.0;
            left_rpm = left_speed / wheel_circumference * 60.0;
        }
    }
}

void CAN_MOT_SBL2360::emgbutton_callback(const std_msgs::msg::Bool::SharedPtr emgbutton_msg)
{
    if(emgbutton_msg->data == true)
    {
        //emgbutton press
        EmgStop();
    }
    else
    {
        EmgRelease();
    }

}

void CAN_MOT_SBL2360::set_encoder_callback(const uni_interface::msg::SetEncoder::SharedPtr set_encoder_msg)
{
    set_encoder_right_value = set_encoder_msg->right_encoder;
    set_encoder_left_value = set_encoder_msg->left_encoder;

    EmgStop();
    rclcpp::sleep_for(std::chrono::milliseconds(500)); // Sleep and wait for motor control stop safely
    write_configure8();

    // Step out emergency stop mode

    rclcpp::sleep_for(std::chrono::milliseconds(500));	// Sleep and wait for motor control operation safely
    EmgRelease();
}

void CAN_MOT_SBL2360::write_configure1()
// torque_constant, numof_pole_pairs, operating mode(0: Trapezoidal, 1: Sinusoidal, 2: Sensorless, 3: AC Induction)
// Swap Windings(0: None, 1: Swapped, 2: Hall only Swapped, Hall+Encoder Swapped)
{
    int ret = 0;

    SetDataMem(MOT_CAN_Tx054_0+0, ((int32_t)torque_constant & 0xffff));
    SetDataMem(MOT_CAN_Tx054_0+1, ((int32_t)torque_constant >> 16));
    SetDataMem(MOT_CAN_Tx054_0+2, ((operating_mode << 8) | numof_pole_pairs));
    SetDataMem(MOT_CAN_Tx054_0+3, ((encoder_count_relative << 8) | swap_winding));

    if(ret != 0)
        motor_Error_publish(5);
}

void CAN_MOT_SBL2360::write_configure2()
// set motor amps limit (5 A * 10), set max speed (rpm) for relative speed commands, set max acceleration rate (200 rpm/s * 10) [0 ~ 500000]
{
    int ret = 0;

    SetDataMem(MOT_CAN_Tx055_0+0, mot_amps_limit);
    SetDataMem(MOT_CAN_Tx055_0+1, max_speed_command);
    SetDataMem(MOT_CAN_Tx055_0+2, (max_acceleration_rate & 0xffff));
    SetDataMem(MOT_CAN_Tx055_0+3, (max_acceleration_rate >> 16));

    if(ret != 0)
        motor_Error_publish(5);
}

void CAN_MOT_SBL2360::write_configure3()
// set max deceleration rate (2000 rpm/s * 10) [0 ~ 500000], Fault Motor Deceleration Rate[0 ~ 500000]
{
    int ret = 0;

    SetDataMem(MOT_CAN_Tx056_0+0, (max_deceleration_rate & 0xffff));
    SetDataMem(MOT_CAN_Tx056_0+1, (max_deceleration_rate >> 16));
    SetDataMem(MOT_CAN_Tx056_0+2, (fault_deceleration_rate & 0xffff));
    SetDataMem(MOT_CAN_Tx056_0+3, (fault_deceleration_rate >> 16));

    if(ret != 0)
        motor_Error_publish(5);
}

void CAN_MOT_SBL2360::write_configure4()
// set PID parameters: rightwheel_gain_p, rightwheel_gain_i
{
    uint32_t tmp_rightwheel_gain_p = (uint32_t)(rightwheel_gain_p * 1000000);
    uint32_t tmp_rightwheel_gain_i = (uint32_t)(rightwheel_gain_i * 1000000);
    int ret = 0;

    SetDataMem(MOT_CAN_Tx057_0+0, (tmp_rightwheel_gain_p & 0xffff));
    SetDataMem(MOT_CAN_Tx057_0+1, (tmp_rightwheel_gain_p >> 16));
    SetDataMem(MOT_CAN_Tx057_0+2, (tmp_rightwheel_gain_i & 0xffff));
    SetDataMem(MOT_CAN_Tx057_0+3, (tmp_rightwheel_gain_i >> 16));

    if(ret != 0)
        motor_Error_publish(5);
}

void CAN_MOT_SBL2360::write_configure5()
// set PID parameters: rightwheel_gain_d, leftwheel_gain_p
{
    uint32_t tmp_rightwheel_gain_d = (uint32_t)(rightwheel_gain_d * 1000000);
    uint32_t tmp_leftwheel_gain_p = (uint32_t)(leftwheel_gain_p * 1000000);
    int ret = 0;

    SetDataMem(MOT_CAN_Tx058_0+0, (tmp_rightwheel_gain_d & 0xffff));
    SetDataMem(MOT_CAN_Tx058_0+1, (tmp_rightwheel_gain_d >> 16));
    SetDataMem(MOT_CAN_Tx058_0+2, (tmp_leftwheel_gain_p & 0xffff));
    SetDataMem(MOT_CAN_Tx058_0+3, (tmp_leftwheel_gain_p >> 16));

    if(ret != 0)
        motor_Error_publish(5);
}

void CAN_MOT_SBL2360::write_configure6()
// set PID parameters: leftwheel_gain_i, leftwheel_gain_d
{
    uint32_t tmp_leftwheel_gain_i = (uint32_t)(leftwheel_gain_i * 1000000);
    uint32_t tmp_leftwheel_gain_d = (uint32_t)(leftwheel_gain_d * 1000000);
    int ret = 0;

    SetDataMem(MOT_CAN_Tx059_0+0, (tmp_leftwheel_gain_i & 0xffff));
    SetDataMem(MOT_CAN_Tx059_0+1, (tmp_leftwheel_gain_i >> 16));
    SetDataMem(MOT_CAN_Tx059_0+2, (tmp_leftwheel_gain_d & 0xffff));
    SetDataMem(MOT_CAN_Tx059_0+3, (tmp_leftwheel_gain_d >> 16));

    if(ret != 0)
        motor_Error_publish(5);
}

void CAN_MOT_SBL2360::write_configure7()
// set motor direction, set encoder counts (ppr), set PWM frequency
{
    int ret = 0;

    SetDataMem(MOT_CAN_Tx05A_0+0, ((mot_direction_L << 8) | mot_direction_R));
    SetDataMem(MOT_CAN_Tx05A_0+1, encoder_ppr);
    SetDataMem(MOT_CAN_Tx05A_0+2, pwm_frequency);
    SetDataMem(MOT_CAN_Tx05A_0+3, 0);

    if(ret != 0)
        motor_Error_publish(5);
}

void CAN_MOT_SBL2360::write_configure8()
{
    int ret = 0;

    SetDataMem(MOT_CAN_Tx05B_0+0, (set_encoder_right_value & 0xffff));
    SetDataMem(MOT_CAN_Tx05B_0+1, (set_encoder_right_value >> 16));
    SetDataMem(MOT_CAN_Tx05B_0+2, (set_encoder_left_value & 0xffff));
    SetDataMem(MOT_CAN_Tx05B_0+3, (set_encoder_left_value >> 16));

    if(ret != 0)
        motor_Error_publish(5);
}

void CAN_MOT_SBL2360::cmdvel_setup()
{
    SafetyStop();

    write_configure1();
    write_configure2();
    write_configure3();
    write_configure4();
    write_configure5();
    write_configure6();
    write_configure7();

    if(emgbutton_sub_on == true)
    {
        RCLCPP_INFO(this->get_logger(),"emgbutton Subscribing to topic %s", emgbutton_sub_topic.c_str());
        emgbutton_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            emgbutton_sub_topic.c_str(), 1, std::bind(&CAN_MOT_SBL2360::emgbutton_callback, this, _1));
    }

    motorbreakoFF();

    RCLCPP_INFO(this->get_logger(),"Subscribing to topic %s", cmdvel_topic.c_str());
    cmdvel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            cmdvel_topic.c_str(), (uint32_t)cmd_vel_quesize, std::bind(&CAN_MOT_SBL2360::cmdvel_callback, this, _1));

    // Subscribe set encoder topic
    set_encoder_sub_ = this->create_subscription<uni_interface::msg::SetEncoder>(
            "/robot/motor/set_encoder", 1, std::bind(&CAN_MOT_SBL2360::set_encoder_callback, this, _1));

}

void CAN_MOT_SBL2360::cmdvel_loop()
{
    int ret = 0;

    SetDataMem(MOT_CAN_Tx050_0+0, (right_rpm & 0xffff));
    SetDataMem(MOT_CAN_Tx050_0+1, (right_rpm >> 16));
    SetDataMem(MOT_CAN_Tx050_0+2, (left_rpm & 0xffff));
    SetDataMem(MOT_CAN_Tx050_0+3, (left_rpm >> 16));

    if(ret != 0)
        motor_Error_publish(5);
}

void CAN_MOT_SBL2360::cmdvel_run()
{
#ifdef _CMDVEL_FORCE_RUN
  // controller.write("!G 1 100\r");
  // controller.write("!G 2 100\r");
  //controller.write("!S 1 10\r");
  //controller.write("!S 2 10\r");
#endif
}

// odom publisher
#ifdef _ODOM_COVAR_SERVER
void CAN_MOT_SBL2360::odom_covar_callback(const uni_interface::RequestOdometryCovariancesRequest& req, uni_interface::RequestOdometryCovariancesResponse& res)
{
  res.odometry_covariances.pose.pose.covariance[0] = 0.000001;
  res.odometry_covariances.pose.pose.covariance[7] = 0.000001;
  res.odometry_covariances.pose.pose.covariance[14] = 1000000;
  res.odometry_covariances.pose.pose.covariance[21] = 1000000;
  res.odometry_covariances.pose.pose.covariance[28] = 1000000;
  res.odometry_covariances.pose.pose.covariance[35] = 0.000001;

  res.odometry_covariances.twist.twist.covariance[0] = 0.000001;
  res.odometry_covariances.twist.twist.covariance[7] = 0.000001;
  res.odometry_covariances.twist.twist.covariance[14] = 1000000;
  res.odometry_covariances.twist.twist.covariance[21] = 1000000;
  res.odometry_covariances.twist.twist.covariance[28] = 1000000;
  res.odometry_covariances.twist.twist.covariance[35] = 0.000001;
}
#endif

/*
  position.pose.covariance =  boost::assign::list_of(1e-3) (0) (0)  (0)  (0)  (0)
                                                       (0) (1e-3)  (0)  (0)  (0)  (0)
                                                       (0)   (0)  (1e6) (0)  (0)  (0)
                                                       (0)   (0)   (0) (1e6) (0)  (0)
                                                       (0)   (0)   (0)  (0) (1e6) (0)
                                                       (0)   (0)   (0)  (0)  (0)  (1e3) ;

  position.twist.covariance =  boost::assign::list_of(1e-3) (0)   (0)  (0)  (0)  (0)
                                                      (0) (1e-3)  (0)  (0)  (0)  (0)
                                                      (0)   (0)  (1e6) (0)  (0)  (0)
                                                      (0)   (0)   (0) (1e6) (0)  (0)
                                                      (0)   (0)   (0)  (0) (1e6) (0)
                                                      (0)   (0)   (0)  (0)  (0)  (1e3) ;

To estimate velocity covariance you should know TICKSMM (128) and SIPCYCLE (100) parameters of your robot (written in your robots flash memory and not accessible with Aria).
First parameter tells you how many encoder impulses (count) gets generated by your robot's forward movement of 1 mm.
Second parameter tells you number of milliseconds between two consecutive Server Information Packets from your robot. The values in the parentheses are for P3-DX (ARCOS).

So an error in determining velocity could come from missing an encoder impulse in a cycle.
This would result in 1/TICKSMM/SIPCYCLE velocity error (mm/ms or m/s) for one wheel.
For P3-DX parameters above, this value is 7.8125e-05. Note that you would err by the same absolute amount of velocity in the next cycle.
Gearbox also plays a role in velocity error, but you would need to measure to find the exact amount.
As a rule of thumb, I would at least double the previous amount in order to include gearbox error.

Now that we have determined maximum error of a single wheel's (transversal) velocity, i.e.
we expect 99.7% of errors to be less than this number, we can determine sigma = max_err/3 and C = sigma^2.
Translational and rotational velocities are determined from left and right wheel velocities like this:

v = (v_R + v_L)/2

w = (v_R - v_L)/(2d)

So the covariance for transversal velocity would be (1/2)^2 2C and the covariance for rotational velocity would be (1/(2d))^2 2C.
The d parameter is 1/DiffConvFactor and is accessible from Aria (ArRobot::getDiffConvFactor()).

*/
void CAN_MOT_SBL2360::odom_Init()
{

    odom_x = 0;
    odom_y = 0;
    odom_yaw = 0;
    odom_last_x = 0;
    odom_last_y = 0;
    odom_last_yaw = 0;

#ifdef _PUB_TF
    tf_msg.header.seq = 0;
    tf_msg.header.frame_id = odom_frame;
    tf_msg.child_frame_id = base_frame;
#endif

    if(odom_pub_on)
    {
        //odom_msg.header.seq = 0;
        odom_msg.header.frame_id = odom_frame;
        odom_msg.child_frame_id = base_frame;

        //odom_msg.pose.covariance.assign(0);
        odom_msg.pose.covariance[0] = 0.000001;
        odom_msg.pose.covariance[7] = 0.000001;
        odom_msg.pose.covariance[14] = 1000000;
        odom_msg.pose.covariance[21] = 1000000;
        odom_msg.pose.covariance[28] = 1000000;
        odom_msg.pose.covariance[35] = 0.000001;

        //odom_msg.twist.covariance.assign(0);
        odom_msg.twist.covariance[0] = 0.000001;
        odom_msg.twist.covariance[7] = 0.000001;
        odom_msg.twist.covariance[14] = 1000000;
        odom_msg.twist.covariance[21] = 1000000;
        odom_msg.twist.covariance[28] = 1000000;
        odom_msg.twist.covariance[35] = 0.000001;
    }

    odom_last_time = millis();
    //current_last_time = millis();
}

void CAN_MOT_SBL2360::publisher_init()
{
    if(current_pub_on)
    {
        RCLCPP_INFO(this->get_logger(),"Publishing to topic robot/motor/ampere");
        current_pub_ = this->create_publisher<uni_interface::msg::Duplex>(current_pub_topic, 10);
        current_msg.header.frame_id = odom_frame;
    }

    if(encoder_pub_on)
    {
        RCLCPP_INFO(this->get_logger(),"Publishing to topic robot/motor/encoder");
        encoder_pub_ = this->create_publisher<uni_interface::msg::Duplex>(encoder_pub_topic, 10);
        encoder_msg.header.frame_id = odom_frame;
    }

    if(power_pub_on)
    {
       RCLCPP_INFO(this->get_logger(),"Publishing to topic robot/motor/power");
       power_pub_ = this->create_publisher<uni_interface::msg::Duplex>(power_pub_topic, 10);
       power_msg.header.frame_id = odom_frame;
    }

    if(robotError_pub_on)
    {
        RCLCPP_INFO(this->get_logger(),"Publishing to topic robot/Error");
        robotError_pub_ = this->create_publisher<uni_interface::msg::RobotError>(robotError_pub_topic, 10);
        robotError_msg.header.frame_id = odom_frame;
        robotError_msg.source_node = "MOTOR";
    }

    if(odom_pub_on)
    {
        RCLCPP_INFO(this->get_logger(),"Publishing to topic %s", odom_topic.c_str());
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
    }
}
void CAN_MOT_SBL2360::odom_setup()
{
#ifdef _PUB_TF
    RCLCPP_INFO(this->get_logger(),"Broadcasting odom tf");
    //odom_broadcaster.init(nh);
#endif



#ifdef _ODOM_COVAR_SERVER
    RCLCPP_INFO(this->get_logger(),"Advertising service on roboteq/odom_covar_srv");
    odom_covar_server = nh.advertiseService("roboteq/odom_covar_srv", &sci_mot_sbl2360::odom_covar_callback, this);
#endif



#ifdef _PUB_TF
    tf_msg.header.seq = 0;
    tf_msg.header.frame_id = odom_frame;
    tf_msg.child_frame_id = base_frame;
#endif



    if(odom_pub_on)
    {
        //odom_msg.header.seq = 0;
        odom_msg.header.frame_id = odom_frame;
        odom_msg.child_frame_id = base_frame;

        //odom_msg.pose.covariance.assign(0);
        odom_msg.pose.covariance[0] = 0.000001;
        odom_msg.pose.covariance[7] = 0.000001;
        odom_msg.pose.covariance[14] = 1000000;
        odom_msg.pose.covariance[21] = 1000000;
        odom_msg.pose.covariance[28] = 1000000;
        odom_msg.pose.covariance[35] = 0.000001;

        //odom_msg.twist.covariance.assign(0);
        odom_msg.twist.covariance[0] = 0.000001;
        odom_msg.twist.covariance[7] = 0.000001;
        odom_msg.twist.covariance[14] = 1000000;
        odom_msg.twist.covariance[21] = 1000000;
        odom_msg.twist.covariance[28] = 1000000;
        odom_msg.twist.covariance[35] = 0.000001;
    }

    odom_last_time = millis();
}

void CAN_MOT_SBL2360::odom_ms_run()
{
    motor_current_publish();
    motor_encoder_publish();
    motor_power_publish();
}

void CAN_MOT_SBL2360::motor_current_publish()
{
    if(current_pub_on)
    {
        current_msg.header.stamp = this->now();
        current_msg.right = (float)Amp_right * 0.1;
        current_msg.left = (float)Amp_left * 0.1;
        current_pub_->publish(current_msg);
    }

}

void CAN_MOT_SBL2360::motor_encoder_publish()
{
    if(encoder_pub_on)
    {
        encoder_msg.header.stamp = this->now();
        encoder_msg.right = (float)odom_encoder_right;
        encoder_msg.left = (float)odom_encoder_left;
        encoder_pub_->publish(encoder_msg);
    }

    if(odom_pub_on)
    {
        odom_publish();
    }


}
void CAN_MOT_SBL2360::motor_power_publish()
{
    if(encoder_pub_on)
    {
        power_msg.header.stamp = this->now();
        power_msg.right = (float)right_power * 0.1;
        power_msg.left = (float)left_power * 0.1;
        power_pub_->publish(power_msg);
    }
}

void CAN_MOT_SBL2360::motor_Error_publish(int error_code)
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
        robotError_msg.header.stamp = this->now();
        switch(error_code){
            case 1:	// CAN initial Com. Error
                robotError_msg.error_type = 1001; // CAN Open Error
                robotError_msg.error_code = error_code + 149;
                robotError_msg.error_msg = "Motor: Initial CAN Com. Error";
                robotError_msg.level = "ERROR";
                RCLCPP_ERROR(this->get_logger(),"Motor: Initial CAN Com. Error");
                break;
            case 2:	// CAN alive Com. Error
                robotError_msg.error_type = 1002; // CAN Read Error
                robotError_msg.error_code = error_code + 149;
                robotError_msg.error_msg = "Motor: CAN Comm Error";
                robotError_msg.level = "ERROR";
                RCLCPP_ERROR(this->get_logger(),"Motor: CAN Comm Error");
                break;
            case 3: // Emg Stop
                robotError_msg.error_type = 150; // Emg
                robotError_msg.error_code = error_code + 149;
                robotError_msg.error_msg = "Motor: Emg Stop emgbutton_callback";
                robotError_msg.level = "INFO";
                RCLCPP_INFO(this->get_logger(),"Motor: Emg Stop emgbutton_callback");
                break;
            case 4: // Emg release
                robotError_msg.error_type = 150; // Emg
                robotError_msg.error_code = error_code + 149;
                robotError_msg.error_msg = "Motor: Emg Release emgbutton_callback";
                robotError_msg.level = "INFO";
                RCLCPP_INFO(this->get_logger(),"Motor: Emg Release emgbutton_callback");
                break;
            case 5: // CAN send data fault
                robotError_msg.error_type = 1003; // CAN Data Loss
                robotError_msg.error_code = error_code + 149;
                robotError_msg.error_msg = "Motor: CAN Send data fault";
                robotError_msg.level = "ERROR";
                RCLCPP_ERROR(this->get_logger(),"Motor: CAN Send data fault");
                break;
            case 6:
                robotError_msg.error_type = 151; // Safety stop
                robotError_msg.error_code = error_code + 149;
                robotError_msg.error_msg = "Motor: SafetyStop";
                robotError_msg.level = "INFO";
                RCLCPP_INFO(this->get_logger(),"Motor: SafetyStop");
                break;
            case 7:
            case 8:
            case 9:
                break;

            //////    FF -  Read Fault Flags
            case 10: // MotorOverheat fault
                robotError_msg.error_type = 152; // MotorOverheat
                robotError_msg.error_code = error_code + 149;
                robotError_msg.error_msg = "Motor Flags: MotorOverheat";
                robotError_msg.level = "ERROR";
                RCLCPP_ERROR(this->get_logger(),"Motor Flags: MotorOverheat");
                break;
            case 11: // MotorOvervoltage fault
                robotError_msg.error_type = 153; // MotorOvervoltage
                robotError_msg.error_code = error_code + 149;
                robotError_msg.error_msg = "Motor Flags: MotorOvervoltage";
                robotError_msg.level = "ERROR";
                RCLCPP_ERROR(this->get_logger(),"Motor Flags: MotorOvervoltage");
                break;
            case 12: // MotorUndervoltage fault
                robotError_msg.error_type = 154; // MotorUndervoltage
                robotError_msg.error_code = error_code + 149;
                robotError_msg.error_msg = "Motor Flags: MotorUndervoltage";
                robotError_msg.level = "ERROR";
                RCLCPP_ERROR(this->get_logger(),"Motor Flags: MotorUndervoltage");
                break;
            case 13: // MotorShortcircuit fault
                robotError_msg.error_type = 155; // MotorShortcircuit
                robotError_msg.error_code = error_code + 149;
                robotError_msg.error_msg = "Motor Flags: MotorShortcircuit";
                robotError_msg.level = "ERROR";
                RCLCPP_ERROR(this->get_logger(),"Motor Flags: MotorShortcircuit");
                break;
            case 14: // MotorEmergencystop fault
                robotError_msg.error_type = 150; // Emg
                robotError_msg.error_code = error_code + 149;
                robotError_msg.error_msg = "Motor Flags: MotorEmergencystop";
                robotError_msg.level = "WARN";
                RCLCPP_INFO(this->get_logger(),"Motor Flags: MotorEmergencystop");
                break;
            case 15: // MotorSetupfault fault
                robotError_msg.error_type = 156; // MotorSetupfault
                robotError_msg.error_code = error_code + 149;
                robotError_msg.error_msg = "Motor Flags: MotorSetupfault";
                robotError_msg.level = "ERROR";
                RCLCPP_ERROR(this->get_logger(),"Motor Flags: MotorSetupfault");
                break;
            case 16: // MotorMOSFETfail fault
                robotError_msg.error_type = 157; // MotorMOSFETfail
                robotError_msg.error_code = error_code + 149;
                robotError_msg.error_msg = "Motor Flags: MotorMOSFETfail";
                robotError_msg.level = "ERROR";
                RCLCPP_ERROR(this->get_logger(),"Motor Flags: MotorMOSFETfail");
                break;
            case 17: // MotorDefaultconfig fault
                robotError_msg.error_type = 158; // MotorDefaultconfig
                robotError_msg.error_code = error_code + 149;
                robotError_msg.error_msg = "Motor Flags: MotorDefaultconfig";
                robotError_msg.level = "'WARN'";
                RCLCPP_WARN(this->get_logger(),"Motor Flags: MotorDefaultconfig");
                break;
			 case 18:
				break;
             case 19: // Motor fault flag clear
                robotError_msg.error_type = 159; // Motor fault flag clear
                robotError_msg.error_code = error_code + 149;
                robotError_msg.error_msg = "Clear Motor fault flag";
                robotError_msg.level = "INFO";
                RCLCPP_INFO(this->get_logger(),"Clear Motor fault flag");
                break;

            //////   FM(1) - Read Runtime Status Flag Right
            case 20: // Amps Limit currently active Right
                robotError_msg.error_type = 160; // Amps Limit currently active Right
                robotError_msg.error_code = error_code + 149;
                robotError_msg.error_msg = "Motor Runtiem Flags Right: Amps Limit";
                robotError_msg.level = "ERROR";
                RCLCPP_ERROR(this->get_logger(),"Motor Runtiem Flags Right: Amps Limit");
                break;
            case 21: // Motor stalled Right
                robotError_msg.error_type = 161; // Motor stalled Right
                robotError_msg.error_code = error_code + 149;
                robotError_msg.error_msg = "Motor Runtiem Flags Right: Motor stalled";
                robotError_msg.level = "ERROR";
                RCLCPP_ERROR(this->get_logger(),"Motor Runtiem Flags Right: Motor stalled");
                break;
            case 22: // Motor Loop Error detected Right
                robotError_msg.error_type = 162; // Motor Loop Error detected Right
                robotError_msg.error_code = error_code + 149;
                robotError_msg.error_msg = "Motor Runtiem Flags Right: Loop Error detected";
                robotError_msg.level = "ERROR";
                RCLCPP_ERROR(this->get_logger(),"Motor Runtiem Flags Right: Loop Error detected");
                break;
            case 23: // Motor Safety Stop active Right
                robotError_msg.error_type = 163; // Motor Safety Stop active Right
                robotError_msg.error_code = error_code + 149;
                robotError_msg.error_msg = "Motor Runtiem Flags Right: Safety Stop";
                robotError_msg.level = "WARN";
                RCLCPP_WARN(this->get_logger(),"Motor Runtiem Flags Right: Safety Stop");
                break;
            case 24: // Motor Forward Limit triggered Right
                robotError_msg.error_type = 164; // Motor Forward Limit triggered Right
                robotError_msg.error_code = error_code + 149;
                robotError_msg.error_msg = "Motor Runtiem Flags Right: Forward Limit";
                robotError_msg.level = "ERROR";
                RCLCPP_ERROR(this->get_logger(),"Motor Runtiem Flags Right: Forward Limit");
                break;
            case 25: // Motor Reverse Limit triggered Right
                robotError_msg.error_type = 165; // Motor Reverse Limit triggered Right
                robotError_msg.error_code = error_code + 149;
                robotError_msg.error_msg = "Motor Runtiem Flags Right: Reverse Limit";
                robotError_msg.level = "ERROR";
                RCLCPP_ERROR(this->get_logger(),"Motor Runtiem Flags Right: Reverse Limit");
                break;
            case 26: // Motor Amps Trigger activated Right
                robotError_msg.error_type = 166; // Motor Amps Trigger activated Right
                robotError_msg.error_code = error_code + 149;
                robotError_msg.error_msg = "Motor Runtiem Flags Right: Amps Trigger";
                robotError_msg.level = "ERROR";
                RCLCPP_ERROR(this->get_logger(),"Motor Runtiem Flags Right: Amps Trigger");
                break;
            case 27:
            case 28:
                break;
            case 29: // Clear Motor Runtime status Right
                robotError_msg.error_type = 169; // Motor Amps Trigger activated Right
                robotError_msg.error_code = error_code + 149;
                robotError_msg.error_msg = "Clear Motor Runtiem Flags Right";
                robotError_msg.level = "INFO";
                RCLCPP_INFO(this->get_logger(),"Clear Motor Runtiem Flags Right");
                break;

            //////   FM(2) - Read Runtime Status Flag Left
            case 30: // Amps Limit currently active Left
                robotError_msg.error_type = 170; // Amps Limit currently active Left
                robotError_msg.error_code = error_code + 149;
                robotError_msg.error_msg = "Motor Runtiem Flags Left: Amps Limit";
                robotError_msg.level = "ERROR";
                RCLCPP_ERROR(this->get_logger(),"Motor Runtiem Flags Left: Amps Limit");
                break;
            case 31: // Motor stalled Left
                robotError_msg.error_type = 171; // Motor stalled Left
                robotError_msg.error_code = error_code + 149;
                robotError_msg.error_msg = "Motor Runtiem Flags Left: Motor stalled";
                robotError_msg.level = "ERROR";
                RCLCPP_ERROR(this->get_logger(),"Motor Runtiem Flags Left: Motor stalled");
                break;
            case 32: // Motor Loop Error detected Left
                robotError_msg.error_type = 172; // Motor Loop Error detected Left
                robotError_msg.error_code = error_code + 149;
                robotError_msg.error_msg = "Motor Runtiem Flags Left: Loop Error detected";
                robotError_msg.level = "ERROR";
                RCLCPP_ERROR(this->get_logger(),"Motor Runtiem Flags Left: Loop Error detected");
                break;
            case 33: // Motor Safety Stop active Left
                robotError_msg.error_type = 173; // Motor Safety Stop active Left
                robotError_msg.error_code = error_code + 149;
                robotError_msg.error_msg = "Motor Runtiem Flags Left: Safety Stop";
                robotError_msg.level = "WARN";
                RCLCPP_WARN(this->get_logger(),"Motor Runtiem Flags Left: Safety Stop");
                break;
            case 34: // Motor Forward Limit triggered Left
                robotError_msg.error_type = 174; // Motor Forward Limit triggered Left
                robotError_msg.error_code = error_code + 149;
                robotError_msg.error_msg = "Motor Runtiem Flags Left: Forward Limit";
                robotError_msg.level = "ERROR";
                RCLCPP_ERROR(this->get_logger(),"Motor Runtiem Flags Left: Forward Limit");
                break;
            case 35: // Motor Reverse Limit triggered Left
                robotError_msg.error_type = 175; // Motor Reverse Limit triggered Left
                robotError_msg.error_code = error_code + 149;
                robotError_msg.error_msg = "Motor Runtiem Flags Left: Reverse Limit";
                robotError_msg.level = "ERROR";
                RCLCPP_ERROR(this->get_logger(),"Motor Runtiem Flags Left: Reverse Limit");
                break;
            case 36: // Motor Amps Trigger activated Left
                robotError_msg.error_type = 176; // Motor Amps Trigger activated Left
                robotError_msg.error_code = error_code + 149;
                robotError_msg.error_msg = "Motor Runtiem Flags Left: Amps Trigger";
                robotError_msg.level = "ERROR";
                RCLCPP_ERROR(this->get_logger(),"Motor Runtiem Flags Left: Amps Trigger");
                break;
            case 37:
            case 38:
                break;
            case 39: // Clear Motor Runtime status Left
                robotError_msg.error_type = 179; // Motor Amps Trigger activated Left
                robotError_msg.error_code = error_code + 149;
                robotError_msg.error_msg = "Clear Motor Runtiem Flags Left";
                robotError_msg.level = "INFO";
                RCLCPP_INFO(this->get_logger(),"Clear Motor Runtiem Flags Left");
                break;

            //////    FS - Read Status Flags
            case 40: // Serial mode
                robotError_msg.error_type = 180; // Serial mode
                robotError_msg.error_code = error_code + 149;
                robotError_msg.error_msg = "Motor Status: Serial mode";
                robotError_msg.level = "WARN";
                RCLCPP_WARN(this->get_logger(),"Motor Status: Serial mode");
                break;
            case 41: // Pulse mode
                robotError_msg.error_type = 181; // Pulse mode
                robotError_msg.error_code = error_code + 149;
                robotError_msg.error_msg = "Motor Status: Pulse mode";
                robotError_msg.level = "INFO";
                RCLCPP_INFO(this->get_logger(),"Motor Status: Pulse mode");
                break;
            case 42: // Analog mode
                robotError_msg.error_type = 182; // Analog mode
                robotError_msg.error_code = error_code + 149;
                robotError_msg.error_msg = "Motor Status: Analog mode";
                robotError_msg.level = "INFO";
                RCLCPP_INFO(this->get_logger(),"Motor Status: Analog mode");
                break;
            case 43: // Power stage off
                robotError_msg.error_type = 183; // Power stage off
                robotError_msg.error_code = error_code + 149;
                robotError_msg.error_msg = "Motor Status: Power stage off";
                robotError_msg.level = "INFO";
                RCLCPP_INFO(this->get_logger(),"Motor Status: Power stage off");
                break;
            case 44: // Stall detected
                robotError_msg.error_type = 184; // Stall detected
                robotError_msg.error_code = error_code + 149;
                robotError_msg.error_msg = "Motor Status: Stall detected";
                robotError_msg.level = "ERROR";
                RCLCPP_ERROR(this->get_logger(),"Motor Status: Stall detected");
                break;
            case 45: // At limit
                robotError_msg.error_type = 185; // At limit
                robotError_msg.error_code = error_code + 149;
                robotError_msg.error_msg = "Motor Status: At limit";
                robotError_msg.level = "ERROR";
                RCLCPP_ERROR(this->get_logger(),"Motor Status: At limit");
                break;
            case 46: // Unused
                robotError_msg.error_type = 186; // Unused
                robotError_msg.error_code = error_code + 149;
                robotError_msg.error_msg = "Motor Status: Unused";
                robotError_msg.level = "INFO";
                RCLCPP_ERROR(this->get_logger(),"Motor Status: Unused");
                break;
            case 47: // MicroBasic script not running
                robotError_msg.error_type = 187; // MicroBasic script not running
                robotError_msg.error_code = error_code + 149;
                robotError_msg.error_msg = "Motor Status: MicroBasic script not running";
                robotError_msg.level = "ERROR";
                RCLCPP_ERROR(this->get_logger(),"Motor Status: MicroBasic script not running");
                break;
            case 48: // Motor/Sensor Tuning mode
                robotError_msg.error_type = 188; // Motor/Sensor Tuning mode
                robotError_msg.error_code = error_code + 149;
                robotError_msg.error_msg = "Motor Status: Motor/Sensor Tuning mode";
                robotError_msg.level = "INFO";
                RCLCPP_INFO(this->get_logger(),"Motor Status: Motor/Sensor Tuning mode");
                break;
            case 49: // Clear Motor Status Flag
                robotError_msg.error_type = 189; // Clear Motor Status Flag
                robotError_msg.error_code = error_code + 149;
                robotError_msg.error_msg = "Clear Motor Status Flags";
                robotError_msg.level = "INFO";
                RCLCPP_INFO(this->get_logger(),"Clear Motor Status Flags");
                break;

            default:
                break;
        }

        robotError_pub_->publish(robotError_msg);
    }

}

void CAN_MOT_SBL2360::odom_publish()
{
    // determine delta time in seconds
    uint32_t nowtime = millis();
    float dt = (float)DELTAT(nowtime,odom_last_time) / 1000.0;
    odom_last_time = nowtime;

    float linear, angular, encoder_cpr_gear;
    float odom_encoder_right_relative, odom_encoder_left_relative;

    encoder_cpr_gear = encoder_cpr * gear_ratio;

    if(encoder_count_relative)
    {
        odom_encoder_right_relative = (float)odom_encoder_right;
        odom_encoder_left_relative = (float)odom_encoder_left;
    }
    else
    {
        odom_encoder_right_relative = (float)odom_encoder_right - (float)odom_encoder_right_prev;
        odom_encoder_left_relative = (float)odom_encoder_left - (float)odom_encoder_left_prev;
    }

    // determine deltas of distance and angle
    linear = (odom_encoder_right_relative / (float)encoder_cpr_gear * wheel_circumference + odom_encoder_left_relative / (float)encoder_cpr_gear * wheel_circumference) /  2.0 ;
    angular = (odom_encoder_right_relative / (float)encoder_cpr_gear * wheel_circumference - odom_encoder_left_relative / (float)encoder_cpr_gear * wheel_circumference) / track_width;

    // Update odometry
    // odom_x += linear * cos(odom_yaw);             // m
    // odom_y += linear * sin(odom_yaw);             // m
    // odom_yaw = NORMALIZE(odom_yaw + angular);     // rad
    odom_x += linear * cos(odom_yaw + angular/2.0);             // m
    odom_y += linear * sin(odom_yaw + angular/2.0);             // m
    odom_yaw += angular;     // rad

    // float vx = (odom_x - odom_last_x) / dt;
    // float vy = (odom_y - odom_last_y) / dt;
    // float vyaw = (odom_yaw - odom_last_yaw) / dt;
    float vx = linear / dt;
    float vy = 0.0;
    float vyaw = angular / dt;

    odom_last_x = odom_x;
    odom_last_y = odom_y;
    odom_last_yaw = odom_yaw;

    tf2::Quaternion q;
    q.setRPY(0, 0, odom_yaw);

    //geometry_msgs::msg::Quaternion quat = toMsg(q);


    rclcpp::Time now = this->now();

    //odom_msg.header.seq++;
    odom_msg.header.stamp = now;
    odom_msg.pose.pose.position.x = odom_x;
    odom_msg.pose.pose.position.y = odom_y;
    odom_msg.pose.pose.position.z = 0.0;
    //odom_msg.pose.pose.orientation = q;

    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = vy;  // vy
    odom_msg.twist.twist.angular.z = vyaw;


    geometry_msgs::msg::TransformStamped odom_tf;

    odom_tf.transform.translation.x = odom_msg.pose.pose.position.x;
    odom_tf.transform.translation.y = odom_msg.pose.pose.position.y;
    odom_tf.transform.translation.z = odom_msg.pose.pose.position.z;
    odom_tf.transform.rotation = odom_msg.pose.pose.orientation;

    odom_tf.header.frame_id = odom_frame;
    odom_tf.child_frame_id = base_frame;
    odom_tf.header.stamp = now;

    odom_pub_->publish(std::move(odom_msg));

    tf_broadcaster_->sendTransform(odom_tf);

}


void CAN_MOT_SBL2360::motorstate_loop()
{
    int comm_error = GetDataMem(MOT_CAN_Rx052_3).s16_data;
    int ret = 0;
    RCLCPP_DEBUG(this->get_logger(),"motorstate_loop()");

    SetDataMem(MOT_CAN_Tx053_0+0, 0);
    SetDataMem(MOT_CAN_Tx053_0+1, 0);
    SetDataMem(MOT_CAN_Tx053_0+2, 0);
    SetDataMem(MOT_CAN_Tx053_0+3, 0);

    if (comm_error)
        motor_Error_publish(2);
    if(ret != 0)
        motor_Error_publish(5);
}


void CAN_MOT_SBL2360::start()
{
    RCLCPP_INFO(this->get_logger(),"Mot CAN Init");
    int ret = 0;

    publisher_init();

	if(ret!=0)
    {
        motor_Error_publish(1);
    }

    odom_setup();
    cmdvel_setup();
}
