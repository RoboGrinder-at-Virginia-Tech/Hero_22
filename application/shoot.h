/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef SHOOT_H
#define SHOOT_H
#include "struct_typedef.h"

#include "CAN_receive.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "user_lib.h"



//射击发射开关通道数据
#define SHOOT_RC_MODE_CHANNEL       1
//云台模式使用的开关通道

#define SHOOT_CONTROL_TIME          GIMBAL_CONTROL_TIME

#define SHOOT_FRIC_PWM_ADD_VALUE    100.0f//200.0f

//射击摩擦轮激光打开 关闭
#define SHOOT_ON_KEYBOARD           KEY_PRESSED_OFFSET_Q
#define SHOOT_OFF_KEYBOARD          KEY_PRESSED_OFFSET_E

//射击完成后 子弹弹出去后，判断时间，以防误触发
#define SHOOT_DONE_KEY_OFF_TIME     15
#define SHOOT_DONE_REVERSE_TIME     1000 //15
//鼠标长按判断 之前80
#define PRESS_LONG_TIME             999

//SZL添加 给鼠标左键用的 鼠标左键长按
#define PRESS_LONG_TIME_L						999

//鼠标右键 长按 
#define PRESS_LONG_TIME_R						999

//键盘v键长按
#define PRESS_LONG_TIME_V						50

//遥控器射击开关打下档一段时间后 连续发射子弹 用于清单
#define RC_S_LONG_TIME              2000
//摩擦轮高速 加速 时间
#define UP_ADD_TIME                 80
//电机反馈码盘值范围
#define HALF_ECD_RANGE              4096
#define ECD_RANGE                   8191
//电机rmp 变化成 旋转速度的比例
//rpm to rad/s
#define MOTOR_RPM_TO_SPEED          0.005511566f
#define MOTOR_ECD_TO_ANGLE          0.00004036791547f
//以下还未改为42mm需要的参数
//42mm 拨弹电机相关参数
#define FULL_COUNT                  18
//拨弹速度
#define TRIGGER_SPEED               10.0f //-10.0f//-5.0f//-10.0f
#define CONTINUE_TRIGGER_SPEED      9.0f //-9.0f Hero42mm没用
#define READY_TRIGGER_SPEED         5.0f //-5.0f Hero42mm没用

#define KEY_OFF_JUGUE_TIME          500
#define SWITCH_TRIGGER_ON           0
#define SWITCH_TRIGGER_OFF          1

//卡单时间 以及反转时间
#define BLOCK_TRIGGER_SPEED         1.0f//1.0f
#define BLOCK_TIME                  800//700
#define REVERSE_TIME                300//150//200//500
#define REVERSE_SPEED_LIMIT         13.0f

#define PI_FOUR                     0.78539816339744830961566084581988f //-0.78539816339744830961566084581988f //没用

//17mm feeder motor控制相关参数
#define MOTOR_RPM_TO_SPEED_17mm  				0.00290888208665721596153948461415f
#define MOTOR_ECD_TO_ANGLE_17mm				 	0.000021305288720633905968306772076277f
#define FULL_COUNT_17mm                  18 //未使用
//拨弹速度
#define TRIGGER_SPEED_17mm               10.0f//-10.0f
#define CONTINUE_TRIGGER_SPEED_17mm      9.0f//-9.0f
#define READY_TRIGGER_SPEED_17mm         5.0f//-5.0f

#define KEY_OFF_JUGUE_TIME_17mm          500
#define SWITCH_TRIGGER_ON_17mm           0
#define SWITCH_TRIGGER_OFF_17mm          1

//卡单时间 以及反转时间
#define BLOCK_TRIGGER_SPEED_17mm         1.0f
#define BLOCK_TIME_17mm                  700
#define REVERSE_TIME_17mm                500
#define REVERSE_SPEED_LIMIT_17mm         13.0f

#define PI_FOUR_17mm                     0.78539816339744830961566084581988f //-0.78539816339744830961566084581988f //没用
//END
/*
Angle calculations for different robot <-> SZL 5-19-2022
弧度制, 范围 (0,2PI], 注意这与 (-PI,PI] 的相位差 不同

Infantry; 拨盘有9个洞, 2pi/9 = 0.698131701f; 为了保证不过冲 set 0.67f
0.57f

Hero; 拨盘3个洞, 2pi/3 = 2.094395102f; 为了保证不过冲 set = 2.05f
2.00

测试用旋转角度180度, 2pi/2 = pi = 3.1415926f; 
1.5PI = 4.712388980f
2.0PI = 6.283185307f

PI_TEN 为角度增量
*/
#define PI_TEN_17mm                      0.55f //-0.55f//-0.60f//0.67f
//2.05f//3.1415926f//0.67f//0.698131701f//3.1415926f//2.094395102f//0.69f//initial 0.314 radian,0.69 is approximately 40 degree

#define PI_TEN_42mm                      2.05f //-2.05f //-2.094395102f //-2.05f //-2.00f//0.67f
#define EJECT_AMMO_ONCE_ANGLE            +1.39f//+1.047197551f
#define AMMO_TURN_BACK_ANGLE             +0.52f

/*仿照云台控制逻辑 新增一个宏定义 电机和转盘安装方向*/
#define TRIG_MOTOR_TURN 1
#define TRIG_MOTOR_42mm_TURN 1
/**
*0 - INF2022
*1 - HERO2022
*/

/*
SZL
Original PID parameter
#define TRIGGER_ANGLE_PID_KP        800.0f//600//800.0f
#define TRIGGER_ANGLE_PID_KI        0.5f//1.0//0.5f
#define TRIGGER_ANGLE_PID_KD        0.0f

#define TRIGGER_BULLET_PID_MAX_OUT  10000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 9000.0f

#define TRIGGER_READY_PID_MAX_OUT   10000.0f
#define TRIGGER_READY_PID_MAX_IOUT  7000.0f
*/
/*Sam老师当时的PID
				bullet trigger motor pid parameter	
  PID_struct_init(&pid_trigger, POSITION_PID, 10000, 2000,
                  KP = 15, KI = 0, KD = 10);
  PID_struct_init(&pid_trigger_spd, POSITION_PID, 8000, 3000,
                  1.5, 0.1, 5);
                  
  kalman_filter_init(&yaw_kalman_filter, &yaw_kalman_filter_para);
  kalman_filter_init(&pitch_kalman_filter, &pitch_kalman_filter_para);
  kalman_filter_init(&dist_kalman_filter, &dist_kalman_filter_para);
	
	void PID_struct_init(
    pid_t*   pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,

    float kp,
    float ki,
    float kd)
*/
//42mm 拨弹轮电机PID 外环PID
#define TRIGGER_ANGLE_PID_42mm_OUTER_KP        20.0f
#define TRIGGER_ANGLE_PID_42mm_OUTER_KI        0.0f
#define TRIGGER_ANGLE_PID_42mm_OUTER_KD        0.0f

#define TRIGGER_BULLET_PID_42mm_OUTER_MAX_OUT  30.0f//25.0f//10.0f
#define TRIGGER_BULLET_PID_42mm_OUTER_MAX_IOUT 1.5f
/*
外环的输出是内环的输入 内环输入单位是rad/s 
*/
//拨弹轮电机PID M3508 42mm
#define TRIGGER_ANGLE_PID_KP        800.0f//100.0f//800.0f//600//800.0f
#define TRIGGER_ANGLE_PID_KI        0.5f//1.0//0.5f
#define TRIGGER_ANGLE_PID_KD        0.0f

#define TRIGGER_BULLET_PID_MAX_OUT  10000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 5000.0f//6-17之前是9000.0f//9000.0f 可以直接初始化成这2个

#define TRIGGER_READY_PID_MAX_OUT   10000.0f
#define TRIGGER_READY_PID_MAX_IOUT  5000.0f//7000.0f 

//17mm 拨弹轮电机PID
#define TRIGGER_ANGLE_PID_KP_17mm        800.0f//100.0f//800.0f//600//800.0f
#define TRIGGER_ANGLE_PID_KI_17mm        0.5f//1.0//0.5f
#define TRIGGER_ANGLE_PID_KD_17mm        0.0f

#define TRIGGER_BULLET_PID_MAX_OUT_17mm  10000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT_17mm 9000.0f//9000.0f 可以直接初始化成这2个

#define TRIGGER_READY_PID_MAX_OUT_17mm   10000.0f
#define TRIGGER_READY_PID_MAX_IOUT_17mm  5000.0f//7000.0f

//42mm
#define SHOOT_HEAT_REMAIN_VALUE     60//90//5-24之前:40//30

//17mm
/*原始值是#define SHOOT_HEAT_REMAIN_VALUE     30*/
#define SHOOT_HEAT_REMAIN_VALUE_17mm     40//60//5-24之前:40//30

/*
12-28-2021 SZL添加 PID M3508 屁股 shooter 电机 2个
发射方向左Left 右Right两个电机，两套变量+宏定义，一般数值保持一样
左为Can ID 1 右为Can ID2
M3508_RIGHT_FRICTION_PID_MAX_OUT = M3508_LEFT_FRICTION_PID_MAX_OUT = TRIGGER_READY_PID_MAX_OUT 约等于 MAX_MOTOR_CAN_CURRENT 16000.0f
*/
//底盘3508最大can发送电流值 16384-->20A
//#define MAX_MOTOR_CAN_CURRENT 16000.0f

//LEFT
#define M3508_LEFT_FRICTION_PID_KP 800.0f
#define M3508_LEFT_FRICTION_PID_KI 10.0f
#define M3508_LEFT_FRICTION_PID_KD 600.0f 

#define M3508_LEFT_FRICTION_PID_MAX_OUT 10000.0f//10000
#define M3508_LEFT_FRICTION_PID_MAX_IOUT 2000.0f

//RIGHT
#define M3508_RIGHT_FRICTION_PID_KP 800.0f //800 //900
#define M3508_RIGHT_FRICTION_PID_KI 10.0f //10 //20
#define M3508_RIGHT_FRICTION_PID_KD 600.0f //600 //600

#define M3508_RIGHT_FRICTION_PID_MAX_OUT 10000.0f
#define M3508_RIGHT_FRICTION_PID_MAX_IOUT 2000.0f

#define M3508_FRIC_MOTOR_RPM_TO_LINEAR_VETOR_SEN 3.141592654e-3f


//SZL 5-15-2022 referee speed limit 
#define INITIAL_PROJECTILE_SPEED_LIMIT_17mm 15
#define INITIAL_PROJECTILE_SPEED_LIMIT_42mm 10

//ICRA 子弹速度上线 为 18m/s
#define ICRA_PROJECTILE_SPEED_LIMIT 18

typedef enum
{
    SHOOT_STOP = 0,
    SHOOT_READY_FRIC,    //1
    SHOOT_READY_BULLET,  //2
    SHOOT_READY,         //3
    SHOOT_BULLET,        //4
    SHOOT_CONTINUE_BULLET,  //5
    SHOOT_DONE,          //6
} shoot_mode_e;

//SZL 12-30-2021 添加 fric 电机 M3508 数据解包 待打包发送数据 结构体
//fric Wheel
typedef struct
{
		fp32 fricW_speed_set;
	  fp32 fricW_speed;
	
    //fp32 fricW_angle;
    //fp32 fricW_set_angle;
    //int8_t fricW_ecd_count;
		
		int16_t fricW_given_current;
} M3508_fric_motor_t;

typedef enum
{
	user_SHOOT_OFF=0,
	user_SHOOT_AUTO, //1
	user_SHOOT_SEMI, //2
}user_fire_ctrl_e;

typedef struct
{
	  uint8_t trigger_motor_17mm_is_online;//0x01=online; 0x00=offline
	  uint8_t trigger_motor_42mm_is_online;//0x01=online; 0x00=offline
	
    shoot_mode_e shoot_mode;//这个是M3508 42mm 的枪管摩擦轮
	
		shoot_mode_e shoot_mode_17mm;//17mm snail 电机的 开火状态
		//SZL 6-10-2022新增
		uint8_t last_key_Q_sts; //0未按下, 1按下
		uint8_t key_Q_cnt;
	
		uint8_t last_key_X_sts;
		uint8_t key_X_cnt;
		uint16_t press_key_X_time;
	
		uint8_t last_key_V_sts;
		uint8_t key_V_cnt;
		uint16_t press_key_V_time; 
	
		user_fire_ctrl_e user_fire_ctrl;//两个共用的, 但42mm无论如何是 单发模式
    const RC_ctrl_t *shoot_rc;
    const motor_measure_t *shoot_motor_measure; //42mm拨弹电机
    ramp_function_source_t fric1_ramp;
    uint16_t fric_pwm1;
    ramp_function_source_t fric2_ramp;
    uint16_t fric_pwm2;
    pid_type_def trigger_motor_pid;//42mm拨盘电机 内环PID
		pid_type_def trigger_motor_angle_pid;//42mm拨盘电机 外环PID
    fp32 trigger_speed_set;//用于需要直接速度控制时的控制速度
    fp32 speed;
    fp32 speed_set;
    fp32 angle;
    fp32 set_angle;
    int16_t given_current;
    int8_t ecd_count;//未使用
		
		//17mm 附发射器
		const motor_measure_t *shoot_motor_measure_17mm; //17mm拨弹电机
		pid_type_def trigger_motor_pid_17mm; //17mm
		fp32 trigger_speed_set_17mm;
    fp32 speed_17mm;
    fp32 speed_set_17mm;
    fp32 angle_17mm;
    fp32 set_angle_17mm;
    int16_t given_current_17mm;
    int8_t ecd_count_17mm;//未使用
		uint8_t key_time_17mm;//看着用
		
    bool_t press_l;
    bool_t press_r;
    bool_t last_press_l;
    bool_t last_press_r;
    uint16_t press_l_time;
    uint16_t press_r_time;
    uint16_t rc_s_time;

    uint16_t block_time;
    uint16_t reverse_time;
    bool_t move_flag;//42mm
		uint8_t block_flag;//42mm堵转标志位
		uint8_t last_block_flag;
		
		bool_t move_flag_turn_back;
		
		uint16_t block_time_17mm;
    uint16_t reverse_time_17mm;
		bool_t move_flag_17mm;//17mm

    bool_t key; //微动开关 PR 屏蔽掉了
    uint32_t key_time;

		uint16_t heat_limit;//42mm的 heat limit
    uint16_t heat;//42mm的 heat

    uint16_t heat_limit_17mm;//17mm的 heat limit
    uint16_t heat_17mm;//17mm的 heat 
		
		/*12-28-2021 SZL add for 
		infantry pid shooter friction wheel LEFT and RIGHT
		Everything above keep the same as the old PWM shooter
		*/
		const motor_measure_t *left_friction_motor_measure;
		const motor_measure_t *right_friction_motor_measure;
		pid_type_def left_fric_motor_pid;
		pid_type_def right_fric_motor_pid;
		
		//LEFT and RIGHT
		M3508_fric_motor_t left_fricMotor;
		M3508_fric_motor_t right_fricMotor;
		
		fp32 currentLeft_speed_set;
		fp32 currentRight_speed_set;
		
		fp32 currentLIM_shoot_speed_42mm;
		fp32 currentLIM_shoot_speed_17mm;
		//当前 摩擦轮PID速度环 输入; 当前规则允许 速度上限 - offset 后 = 这个数
		//所以 上面这个数 + offset = 预计速度
		
		fp32 predict_shoot_speed;//for CV
		
		uint16_t referee_current_shooter_17mm_speed_limit;
		uint16_t referee_current_shooter_42mm_speed_limit;
		
		uint8_t ammoBox_sts;
		
} shoot_control_t;

//shoot motor 是 拨弹轮 M2006 motor


extern void shoot_init(void);
extern int16_t shoot_control_loop(void);

#endif
