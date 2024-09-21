#include "Driver_FMU.h"
#include <ardupilotmega/mavlink.h>

#define MAVLINK_COM COM1  // 接mavlink协议的串口
#define MAVLINK_SYSTEM_ID 1
#define MAVLINK_COMPONENT_ID 0

// 全局变量
bool already_got_heartbeat = false;  // 是否已经收到mavlink心跳帧
bool already_request_mav_stream = false;  // 是否已经请求数据流

mavlink_heartbeat_t heartbeat;
mavlink_gps_raw_int_t gps_raw_int;
mavlink_attitude_t attitude;
mavlink_global_position_int_t global_position_int;

FREQ_calc_t freq_calc_heartbeat;
FREQ_calc_t freq_calc_gps_raw_int;
FREQ_calc_t freq_calc_attitude;
FREQ_calc_t


/// @brief 更新频率计算结构体
/// @param p_freq_calc 频率计算结构体
void update_freq_calc(FREQ_calc_t *p_freq_calc)
{
	// 计算当前时间与上次获取帧时间的时间差，单位为毫秒
	p_freq_calc->period_ms = xTaskGetTickCount() - p_freq_calc->last_got_frame_time_ms;

	// 如果时间差为0，说明在同一时间周期内，频率设为0
	if (p_freq_calc->period_ms == 0)
	{
		p_freq_calc->freq_Hz = 0;
		
	}
	else
	{
		// 否则，根据时间差计算频率，单位为赫兹
		p_freq_calc->freq_Hz = 1000.0f / (float)p_freq_calc->period_ms;
	}

	// 更新上次获取帧时间的时间戳
	p_freq_calc->last_got_frame_time_ms = xTaskGetTickCount();
}

uint8_t current_flight_mode = 0;  // 当前飞行模式
        // STABILIZE =     0,  // manual airframe angle with manual throttle
        // ACRO =          1,  // manual body-frame angular rate with manual throttle
        // ALT_HOLD =      2,  // manual airframe angle with automatic throttle
        // AUTO =          3,  // fully automatic waypoint control using mission commands
        // GUIDED =        4,  // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
        // LOITER =        5,  // automatic horizontal acceleration with automatic throttle
        // RTL =           6,  // automatic return to launching point
        // CIRCLE =        7,  // automatic circular flight with automatic throttle
        // LAND =          9,  // automatic landing with horizontal position control
        // DRIFT =        11,  // semi-autonomous position, yaw and throttle control
        // SPORT =        13,  // manual earth-frame angular rate control with manual throttle
        // FLIP =         14,  // automatically flip the vehicle on the roll axis
        // AUTOTUNE =     15,  // automatically tune the vehicle's roll and pitch gains
        // POSHOLD =      16,  // automatic position hold with manual override, with automatic throttle
        // BRAKE =        17,  // full-brake using inertial/GPS system, no pilot input
        // THROW =        18,  // throw to launch mode using inertial/GPS system, no pilot input
        // AVOID_ADSB =   19,  // automatic avoidance of obstacles in the macro scale - e.g. full-sized aircraft
        // GUIDED_NOGPS = 20,  // guided mode but only accepts attitude and altitude
        // SMART_RTL =    21,  // SMART_RTL returns to home by retracing its steps
        // FLOWHOLD  =    22,  // FLOWHOLD holds position with optical flow without rangefinder
        // FOLLOW    =    23,  // follow attempts to follow another vehicle or ground station
        // ZIGZAG    =    24,  // ZIGZAG mode is able to fly in a zigzag manner with predefined point A and point B
        // SYSTEMID  =    25,  // System ID mode produces automated system identification signals in the controllers
        // AUTOROTATE =   26,  // Autonomous autorotation
        // AUTO_RTL =     27,  // Auto RTL, this is not a true mode, AUTO will report as this mode if entered to perform a DO_LAND_START Landing sequence
        // TURTLE =       28,  // Flip over after crash

uint8_t gnss_fix_type = 0;  // 当前定位状态
        // NO_GPS = 0,                  // No GPS connected/detected
        // NO_FIX = 1,                  // Receiving valid GPS messages but no lock
        // GPS_OK_FIX_2D = 2,           // Receiving valid messages and 2D lock
        // GPS_OK_FIX_3D = 3,           // Receiving valid messages and 3D lock
        // GPS_OK_FIX_3D_DGPS = 4,      // Receiving valid messages and 3D lock with differential improvements
        // GPS_OK_FIX_3D_RTK_FLOAT = 5, // Receiving valid messages and 3D RTK Float
        // GPS_OK_FIX_3D_RTK_FIXED = 6, // Receiving valid messages and 3D RTK Fixed

uint8_t gnss_sat_num = 0;  // 参与定位卫星数
uint16_t gnss_hdop = 0xFFFF;  // GNSS 水平定位因子

float roll_deg = 0;  // 横滚角，单位：度
float pitch_deg = 0;  // 俯仰角，单位：度
float yaw_deg = 0;  // 航向角，单位：度，范围：-180度~180度。正北为0，正东为90度，正西为-90度

int32_t latitude = 0;  // 维度，单位：原始值（度）乘以10的7次方
int32_t longitude = 0;  // 经度，单位：原始值（度）乘以10的7次方

int32_t alt_msl_mm = 0;  // 海拔高度，单位：mm
int32_t alt_above_home_mm = 0;  // 相对于home点的高度，单位：mm

int16_t speed_north_cm_s = 0;  // 北向速度，单位：cm/s
int16_t speed_east_cm_s = 0;  // 东向速度，单位：cm/s
int16_t speed_down_cm_s = 0;  // 垂直速度，单位：cm/s，注意，竖直向下为正

// 函数原型
void request_mavlink_stream(uint8_t req_stream_id, uint16_t req_message_rate);

/// @brief 将弧度转换为角度
/// @param rad 弧度值
/// @return 角度值
float degrees(float rad)
{
    return rad * (180.0f / 3.14159265f);
}

/// @brief 飞控接口初始化
/// @param  空
void FMU_Init(void)
{
	Init_COM_with_DMA_idle(&COM1, "COM1", &huart1, &hdma_usart1_rx, &FMU_decode);
}

mavlink_status_t status;  // mavlink解析状态。注意，此变量不可以定义为局部变量，否则无法记下当前的解析状态
mavlink_message_t rec_mav_msg;  // 存放接收到的mavlink信息
int chan = MAVLINK_COMM_0;  // mavlink通道

/// @brief 解析飞控发送来的数据
/// @param data_in 接收到的数据
void FMU_decode(uint8_t data_in)
{
	if (mavlink_parse_char(chan, data_in, &rec_mav_msg, &status))
	{
		switch (rec_mav_msg.msgid)
		{
		case MAVLINK_MSG_ID_HEARTBEAT:
			SEGGER_RTT_printf(0, RTT_CTRL_TEXT_RED "Got one heartbeat frame\r\n");
			mavlink_msg_heartbeat_decode(&rec_mav_msg, &heartbeat);
			current_flight_mode = heartbeat.custom_mode;  // 心跳帧中包含当前的飞行模式
			already_got_heartbeat = true;
			update_freq_calc(&freq_calc_heartbeat);
			break;

		case MAVLINK_MSG_ID_GPS_RAW_INT:
			mavlink_msg_gps_raw_int_decode(&rec_mav_msg, &gps_raw_int);
			gnss_fix_type = gps_raw_int.fix_type;
			gnss_sat_num = gps_raw_int.satellites_visible;
			gnss_hdop = gps_raw_int.eph;
			update_freq_calc(&freq_calc_gps_raw_int);
			break;

		case MAVLINK_MSG_ID_ATTITUDE:
			mavlink_msg_attitude_decode(&rec_mav_msg, &attitude);
			roll_deg = degrees(attitude.roll);
			pitch_deg = degrees(attitude.pitch);
			yaw_deg = degrees(attitude.yaw);
			update_freq_calc(&freq_calc_attitude);
			break;

		case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
			mavlink_msg_global_position_int_decode(&rec_mav_msg, &global_position_int);
			latitude = global_position_int.lat;
			longitude = global_position_int.lon;
			alt_msl_mm = global_position_int.alt;
			alt_above_home_mm = global_position_int.relative_alt;
			speed_north_cm_s = global_position_int.vx;
			speed_east_cm_s = global_position_int.vy;
			speed_down_cm_s = global_position_int.vz;
			update_freq_calc(&freq_calc_global_position_int);
			break;
		
		default:
			// SEGGER_RTT_printf(0, RTT_CTRL_TEXT_RED "Got one frame, ID: %X\r\n", rec_mav_msg.msgid);
			break;
		}
	}
}

/// @brief 通过串口发送mavlink帧
/// @param message 要发送的mavlink帧
void send_mavlink_msg(const mavlink_message_t *msg)
{
	uint8_t buf[300];

	// Translate message to buffer
	uint16_t len = mavlink_msg_to_send_buffer(buf, msg);

	COM_send_data(&MAVLINK_COM, buf, len);
}

/// @brief 向mavlink设备请求数据流
/// @param req_stream_id 数据流的ID
/// @param req_message_rate 数据流的频率，单位Hz. 如果要关闭此流，则此值设置为0
void request_mavlink_stream(uint8_t req_stream_id, uint16_t req_message_rate)
{
	mavlink_message_t message;
	mavlink_request_data_stream_t rds;
	rds.req_message_rate = req_message_rate;
	rds.target_system = MAVLINK_SYSTEM_ID;
	rds.target_component = MAVLINK_COMPONENT_ID;
	rds.req_stream_id = req_stream_id;
	rds.start_stop = (req_message_rate == 0) ? 0 : 1;
	mavlink_msg_request_data_stream_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &message, &rds);
	send_mavlink_msg(&message);
}

void FMU_task(void *argument)
{
	FMU_Init();

	TickType_t xLastWakeTime = xTaskGetTickCount();

	for (;;)
	{
		if (already_got_heartbeat)  // 已经收到心跳帧
		{
			if (already_request_mav_stream == false)  // 还未请求数据流
			{
				request_mavlink_stream(MAV_DATA_STREAM_EXTENDED_STATUS, 2);
				request_mavlink_stream(MAV_DATA_STREAM_EXTRA1, 5);
				request_mavlink_stream(MAV_DATA_STREAM_POSITION, 5);
				already_request_mav_stream = true;
			}
		}

		vTaskDelayUntil(&xLastWakeTime, 2);
	}
}
