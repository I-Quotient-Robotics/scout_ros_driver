Header header

int8 MOTOR_ID_FRONT_RIGHT = 0
int8 MOTOR_ID_FRONT_LEFT = 1
int8 MOTOR_ID_REAR_RIGHT = 2
int8 MOTOR_ID_REAR_LEFT = 3

int8 LIGHT_ID_FRONT = 0
int8 LIGHT_ID_REAR = 1

# base state
uint8 base_state
uint8 control_mode
uint8 error_code
float64 battery_voltage

float32 vx
float32 vth


# driver state
ScoutDriverState[4] driver_states

ScoutLightState light_state

ScoutOdom odom

ScoutVersion version

ScoutTeleop teleop

ScoutBMS bms


# motor state
ScoutMotorState[4] motor_states