#ifndef REFEREE_H
#define REFEREE_H

#include "main.h"

#include "protocol.h"

//SZL更具 串口协议附录 V1.3 加了新的机器人类型 和 操作手类型
typedef enum
{
    RED_HERO        = 1,
    RED_ENGINEER    = 2,
    RED_STANDARD_1  = 3,
    RED_STANDARD_2  = 4,
    RED_STANDARD_3  = 5,
    RED_AERIAL      = 6,
    RED_SENTRY      = 7,
		RED_DART				= 8,
		RED_RADAR 			= 9,
    BLUE_HERO       = 101,
    BLUE_ENGINEER   = 102,
    BLUE_STANDARD_1 = 103,
    BLUE_STANDARD_2 = 104,
    BLUE_STANDARD_3 = 105,
    BLUE_AERIAL     = 106,
    BLUE_SENTRY     = 107,
		BLUE_DART				= 108,
		BLUE_RADAR 			= 109,
} robot_id_t;

typedef enum
{
    OPERATOR_RED_HERO        = 0x0101,
    OPERATOR_RED_ENGINEER    = 0x0102,
    OPERATOR_RED_STANDARD_1  = 0x0103,
    OPERATOR_RED_STANDARD_2  = 0x0104,
    OPERATOR_RED_STANDARD_3  = 0x0105,
    OPERATOR_RED_AERIAL      = 0x0106,
    
		//
    OPERATOR_BLUE_HERO       = 0x0165,
    OPERATOR_BLUE_ENGINEER   = 0x0166,
    OPERATOR_BLUE_STANDARD_1 = 0x0167,
    OPERATOR_BLUE_STANDARD_2 = 0x0168,
    OPERATOR_BLUE_STANDARD_3 = 0x0169,
    OPERATOR_BLUE_AERIAL     = 0x016A,
    
} operator_robot_id_t;

typedef enum
{
    PROGRESS_UNSTART        = 0,
    PROGRESS_PREPARE        = 1,
    PROGRESS_SELFCHECK      = 2,
    PROGRESS_5sCOUNTDOWN    = 3,
    PROGRESS_BATTLE         = 4,
    PROGRESS_CALCULATING    = 5,
} game_progress_t;
typedef __packed struct //0001
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
} ext_game_state_t;

typedef __packed struct //0002
{
    uint8_t winner;
} ext_game_result_t;
typedef __packed struct //0003 SZL 5-14-2022
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_base_HP;
} ext_game_robot_HP_t;
/*
0x0005暂时没有 ICRA不是人工智能挑战赛 不需要这部分信息
*/
typedef __packed struct //0101
{
    uint32_t event_type;
} ext_event_data_t;

typedef __packed struct //0x0102
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;


typedef __packed struct //0x0103
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_num;
} ext_supply_projectile_booking_t;

typedef __packed struct
{
    uint8_t level;
    uint8_t foul_robot_id;
} ext_referee_warning_t;

/*0x0104裁判系统判罚信息 也暂且不需要
0x0105 飞镖相关
*/
//----------------------------
/*
新增 SZL 5-14-2022
*/
typedef __packed struct
{
uint8_t robot_id;
uint8_t robot_level;
uint16_t remain_HP;
uint16_t max_HP;
uint16_t shooter_id1_17mm_cooling_rate;
uint16_t shooter_id1_17mm_cooling_limit;
uint16_t shooter_id1_17mm_speed_limit;
uint16_t shooter_id2_17mm_cooling_rate;
uint16_t shooter_id2_17mm_cooling_limit;
uint16_t shooter_id2_17mm_speed_limit;
uint16_t shooter_id1_42mm_cooling_rate;
uint16_t shooter_id1_42mm_cooling_limit;
uint16_t shooter_id1_42mm_speed_limit;
uint16_t chassis_power_limit;
uint8_t mains_power_gimbal_output : 1;
uint8_t mains_power_chassis_output : 1;
uint8_t mains_power_shooter_output : 1;
} ext_game_robot_status_t;

/*
下面这个是判断枪口热量用了的:
原先的:
typedef __packed struct //0x0202
{
    uint16_t chassis_volt;
    uint16_t chassis_current;
    float chassis_power;
    uint16_t chassis_power_buffer;
    uint16_t shooter_heat0;
    uint16_t shooter_heat1;
} ext_power_heat_data_t;
shooter_heat0 对应 shooter_id1_17mm_cooling_heat
shooter_heat1 对应 shooter_id2_17mm_cooling_heat
*/
typedef __packed struct //0x0202 SZL改为串口协议附录V1.3
{
    uint16_t chassis_volt;
		uint16_t chassis_current;
		float chassis_power;
		uint16_t chassis_power_buffer;
		uint16_t shooter_id1_17mm_cooling_heat;
		uint16_t shooter_id2_17mm_cooling_heat;
		uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;

typedef __packed struct //0x0203
{
    float x;
    float y;
    float z;
    float yaw;
} ext_game_robot_pos_t;

typedef __packed struct //0x0204
{
    uint8_t power_rune_buff;
} ext_buff_musk_t;

typedef __packed struct //0x0205
{
    uint8_t energy_point;
    uint8_t attack_time;
} aerial_robot_energy_t;

typedef __packed struct //0x0206
{
    uint8_t armor_type : 4;
    uint8_t hurt_type : 4;
} ext_robot_hurt_t;

typedef __packed struct //0x0207
{
    uint8_t bullet_type;
		uint8_t shooter_id;//新增的
    uint8_t bullet_freq;
    float bullet_speed;
} ext_shoot_data_t;

typedef __packed struct //0x0208 RMUL 金币相关 SZL改为串口协议附录V1.3
{
    uint16_t bullet_remaining_num_17mm;
		uint16_t bullet_remaining_num_42mm;
		uint16_t coin_remaining_num;
} ext_bullet_remaining_t;
//0x0209 RFID状态 暂且不需要
//0x020A 飞镖机器人客户端指令数据 暂且不需要

//SZL改为串口协议附录V1.3l 新的交互数据接收信息
typedef __packed struct //0x0301
{
    uint16_t data_cmd_id;
		uint16_t sender_ID;
		uint16_t receiver_ID;
} ext_student_interactive_data_t;

typedef __packed struct
{
    float data1;
    float data2;
    float data3;
    uint8_t data4;
} custom_data_t;


typedef __packed struct
{
    uint8_t data[64];
} ext_up_stream_data_t;

typedef __packed struct
{
    uint8_t data[32];
} ext_download_stream_data_t;


extern void init_referee_struct_data(void);
extern void referee_data_solve(uint8_t *frame);

extern void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer);

extern uint8_t get_robot_id(void);
extern uint8_t get_robot_level(void);

extern void get_shooter_id1_17mm_heat_limit_and_heat(uint16_t *heat1_limit, uint16_t *heat1);
extern void get_shooter_id2_17mm_heat_limit_and_heat(uint16_t *heat1_limit, uint16_t *heat1);
extern uint16_t get_chassis_power_limit(void);
extern uint16_t get_shooter_id1_17mm_speed_limit(void);
extern uint16_t get_shooter_id2_17mm_speed_limit(void);
extern uint16_t get_shooter_id1_42mm_speed_limit(void);
extern void get_shooter_id1_42mm_heat_limit_and_heat(uint16_t *heat0_limit, uint16_t *heat0);
#endif
