std_msgs/Header header

int8 MOTOR_ID_FRONT_RIGHT = 0
int8 MOTOR_ID_FRONT_LEFT = 1
int8 MOTOR_ID_REAR_RIGHT = 2
int8 MOTOR_ID_REAR_LEFT = 3

int8 LIGHT_ID_FRONT = 0
int8 LIGHT_ID_REAR = 1

# motion state
float64 linear_velocity
float64 angular_velocity

# base state
uint8 vehicle_state
uint8 control_mode
uint16 error_code
float64 battery_voltage

# BMS 数据（协议 v2）。部分固件/车型可能不发送 BMS 帧，此时 soc/soh 为 0，
# 上层应把它当作“未知”，不要显示成 0%。
uint8 battery_soc           # 0-100 %
uint8 battery_soh           # 0-100 %
float32 battery_current     # A，正=放电，负=充电（BMS 约定）
float32 battery_temperature # 摄氏度

# motor state
BunkerActuatorState[2] actuator_states

# bool light_control_enabled
# TracerLightState front_light_state

#odometer state
# float64 left_odomter
# float64 right_odomter
