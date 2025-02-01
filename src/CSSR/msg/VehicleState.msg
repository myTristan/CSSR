## timestamp, ros time or utc time
float64 timestamp 

## task related
int32 	id
string  name
int16 	vehicle_state #0: normal 1: damage 2: low battery 
int16 	type
#发布自车71
##未知 0 坦克 10 装甲运兵车 11 轻型高机动平台12 小型无人战车 13 机器狗 14 保障车15
##YHS 17 LT 18 Lynx 19

## RTK state
string 	RTK_state_string
int64   RTK_seq_num
int64   RTK_timestamp_sec
int64   RTK_timestamp_nsec

int16   RTK_gps_status
int16   RTK_gps_service


## RTK position
float64 RTK_gps_longitude   #degree
float64 RTK_gps_latitude    #degree
float64 RTK_gps_altitude    #m

##position of c.g.
float64 RTK_gps_UTM_x       #meter
float64 RTK_gps_UTM_y       #meter
float64 RTK_gps_UTM_z       #meter


##RTK ENU speed, m/s, global coordinate system
float64 RTK_linear_ENU_vx
float64 RTK_linear_ENU_vy
float64 RTK_linear_ENU_vz

##RTK local coordinate system, m/s
float64 RTK_linear_vx
float64 RTK_linear_vy
float64 RTK_linear_vz

##RTK local angular speed, m/s
float64 RTK_angular_vx
float64 RTK_angular_vy
float64 RTK_angular_vz

##key vehicle state
##position, meter
float64 x
float64 y
float64 z

##RTK local coordinate system, m/s
float64 speed_x
float64 speed_y
float64 speed_z

## acceleration m/s2
float64 acc_x
float64 acc_y
float64 acc_z

## rad
float64 heading         #yaw, rad
float64 pitch           #rad
float64 roll            #rad
float64 yaw_rate        #rad/s

##vehicle size
float64 size_x          #meter
float64 size_y          #meter
float64 size_z          #meter
float64 wheelbase       #meter

##gear position
#GEAR_NONE       (0)
#GEAR_PARK       (1)
#GEAR_REVERSE    (2)
#GEAR_NEUTRAL    (3)
#GEAR_DRIVE      (4)
#GEAR_LOW        (5)
int16   gear_pos


#by-wire, true or false
bool    by_wire_enabled
int16   vehicle_mode         # vehicle mode feedback 0：remote mode, 1：self driving mode


## vehicle state from CAN bus
float64 throttle_cmd
float64 throttle_input
float64 throttle_state      #percentage %
bool    throttle_enabled
bool    throttle_override
bool    throttle_driver
bool    throttle_timeout

float64 brake_cmd
float64 brake_input
float64 brake_state         #percentage %
float64 brake_torq_cmd
float64 brake_torq_input
float64 brake_torq_state
bool    brake_boo_output
bool    brake_enabled
bool    brake_override
bool    brake_driver
bool    brake_timeout

float64 steer_cmd
float64 steer_state_front_wheel # rad
float64 steer_torque
bool    steer_enabled
bool    steer_override
bool    steer_timeout


## Batter and Engine
float64 battery_volt        # V
float64 battery_SOC         # %
int16 engine_speed          # r/min

int16 vehicle_bug_code
int16 vehicle_state_code 

