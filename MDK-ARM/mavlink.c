#include "mavlink.h"
#include <ardupilotmega/mavlink.h>

#define MAVLINK_COM COM1  // ��mavlinkЭ��Ĵ���
#define MAVLINK_SYSTEM_ID 1
#define MAVLINK_COMPONENT_ID 0

// ȫ�ֱ���
bool already_got_heartbeat = false;  // �Ƿ��Ѿ��յ�mavlink����֡
bool already_request_mav_stream = false;  // �Ƿ��Ѿ�����������

mavlink_heartbeat_t heartbeat;
mavlink_gps_raw_int_t gps_raw_int;
mavlink_attitude_t attitude;
mavlink_global_position_int_t global_position_int;

FREQ_calc_t freq_calc_heartbeat;
FREQ_calc_t freq_calc_gps_raw_int;
FREQ_calc_t freq_calc_attitude;
FREQ_calc_t


/// @brief ����Ƶ�ʼ���ṹ��
/// @param p_freq_calc Ƶ�ʼ���ṹ��
void update_freq_calc(FREQ_calc_t *p_freq_calc)
{
	// ���㵱ǰʱ�����ϴλ�ȡ֡ʱ���ʱ����λΪ����
	p_freq_calc->period_ms = xTaskGetTickCount() - p_freq_calc->last_got_frame_time_ms;

	// ���ʱ���Ϊ0��˵����ͬһʱ�������ڣ�Ƶ����Ϊ0
	if (p_freq_calc->period_ms == 0)
	{
		p_freq_calc->freq_Hz = 0;
		
	}
	else
	{
		// ���򣬸���ʱ������Ƶ�ʣ���λΪ����
		p_freq_calc->freq_Hz = 1000.0f / (float)p_freq_calc->period_ms;
	}

	// �����ϴλ�ȡ֡ʱ���ʱ���
	p_freq_calc->last_got_frame_time_ms = xTaskGetTickCount();
}

uint8_t current_flight_mode = 0;  // ��ǰ����ģʽ
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

uint8_t gnss_fix_type = 0;  // ��ǰ��λ״̬
        // NO_GPS = 0,                  // No GPS connected/detected
        // NO_FIX = 1,                  // Receiving valid GPS messages but no lock
        // GPS_OK_FIX_2D = 2,           // Receiving valid messages and 2D lock
        // GPS_OK_FIX_3D = 3,           // Receiving valid messages and 3D lock
        // GPS_OK_FIX_3D_DGPS = 4,      // Receiving valid messages and 3D lock with differential improvements
        // GPS_OK_FIX_3D_RTK_FLOAT = 5, // Receiving valid messages and 3D RTK Float
        // GPS_OK_FIX_3D_RTK_FIXED = 6, // Receiving valid messages and 3D RTK Fixed

uint8_t gnss_sat_num = 0;  // ���붨λ������
uint16_t gnss_hdop = 0xFFFF;  // GNSS ˮƽ��λ����

float roll_deg = 0;  // ����ǣ���λ����
float pitch_deg = 0;  // �����ǣ���λ����
float yaw_deg = 0;  // ����ǣ���λ���ȣ���Χ��-180��~180�ȡ�����Ϊ0������Ϊ90�ȣ�����Ϊ-90��

int32_t latitude = 0;  // ά�ȣ���λ��ԭʼֵ���ȣ�����10��7�η�
int32_t longitude = 0;  // ���ȣ���λ��ԭʼֵ���ȣ�����10��7�η�

int32_t alt_msl_mm = 0;  // ���θ߶ȣ���λ��mm
int32_t alt_above_home_mm = 0;  // �����home��ĸ߶ȣ���λ��mm

int16_t speed_north_cm_s = 0;  // �����ٶȣ���λ��cm/s
int16_t speed_east_cm_s = 0;  // �����ٶȣ���λ��cm/s
int16_t speed_down_cm_s = 0;  // ��ֱ�ٶȣ���λ��cm/s��ע�⣬��ֱ����Ϊ��

// ����ԭ��
void request_mavlink_stream(uint8_t req_stream_id, uint16_t req_message_rate);

/// @brief ������ת��Ϊ�Ƕ�
/// @param rad ����ֵ
/// @return �Ƕ�ֵ
float degrees(float rad)
{
    return rad * (180.0f / 3.14159265f);
}

/// @brief �ɿؽӿڳ�ʼ��
/// @param  ��
void FMU_Init(void)
{
	Init_COM_with_DMA_idle(&COM1, "COM1", &huart1, &hdma_usart1_rx, &FMU_decode);
}

mavlink_status_t status;  // mavlink����״̬��ע�⣬�˱��������Զ���Ϊ�ֲ������������޷����µ�ǰ�Ľ���״̬
mavlink_message_t rec_mav_msg;  // ��Ž��յ���mavlink��Ϣ
int chan = MAVLINK_COMM_0;  // mavlinkͨ��

/// @brief �����ɿط�����������
/// @param data_in ���յ�������
void FMU_decode(uint8_t data_in)
{
	if (mavlink_parse_char(chan, data_in, &rec_mav_msg, &status))
	{
		switch (rec_mav_msg.msgid)
		{
		case MAVLINK_MSG_ID_HEARTBEAT:
			SEGGER_RTT_printf(0, RTT_CTRL_TEXT_RED "Got one heartbeat frame\r\n");
			mavlink_msg_heartbeat_decode(&rec_mav_msg, &heartbeat);
			current_flight_mode = heartbeat.custom_mode;  // ����֡�а�����ǰ�ķ���ģʽ
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

/// @brief ͨ�����ڷ���mavlink֡
/// @param message Ҫ���͵�mavlink֡
void send_mavlink_msg(const mavlink_message_t *msg)
{
	uint8_t buf[300];

	// Translate message to buffer
	uint16_t len = mavlink_msg_to_send_buffer(buf, msg);

	COM_send_data(&MAVLINK_COM, buf, len);
}

/// @brief ��mavlink�豸����������
/// @param req_stream_id ��������ID
/// @param req_message_rate ��������Ƶ�ʣ���λHz. ���Ҫ�رմ��������ֵ����Ϊ0
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
		if (already_got_heartbeat)  // �Ѿ��յ�����֡
		{
			if (already_request_mav_stream == false)  // ��δ����������
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
