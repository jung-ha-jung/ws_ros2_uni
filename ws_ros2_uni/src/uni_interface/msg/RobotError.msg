# 
std_msgs/Header header
int32 error_type	# Definition of error type.
                    # 1000~: Common Error type. 
                    # 1000: Clear
                    # 1001: Serial Open Error, 1002: Serial Read Error, 1003: Serial Data Loss
                    # 1101: Ethernet Open Error, 1102: Ethernet Read Error, 1103: Ethernet Data Loss
                    # 1201: CAN Open Error, 1202: CAN Read Error, 1203: CAN Data Loss
                    # 0~999: Each Node has speical error type. -> 0~49: IMU / 50~99: LiDAR / 100~149: BAT / 150~199: Motor / 200~249: GPIO(EMG) / 250~299: STM32 / 300~349: STATE
int32 error_code	# Error code line indication number.
                    # 0~49: IMU / 50~99: LiDAR / 100~149: BAT / 150~199: Motor / 200~249: GPIO(EMG) / 250~299: STM32 / 300~349: STATE
                    # 0: IMU Open OK
string error_msg	# "Error message description"
string source_node	# IMU, LIDAR, BAT, MOTOR, BAT, GPIO(EMG), STM32
string level		# WARN, ERROR, INFO
