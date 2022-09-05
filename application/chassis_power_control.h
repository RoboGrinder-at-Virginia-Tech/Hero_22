/**
  ****************************(C) COPYRIGHT 2022 RoboGrinder at Virginia Tech****************************
  * @file       chassis_power_control.c/h
  * @brief      chassis power control.底盘功率控制
  * @note       Based on competition rules: chassis power control.
  *             Two ways to limit chassis power 
	*							1) The overall program structure is from DJI 2019 example code and file "chassis_power_control"
	*									based on remaining buffer # to regulate and to control the raw esc control current(message send to can bus)
	*             2) Speed adaptive chassis power control; the program will regulate and control the PID speed of each wheel first.
  *           
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. add chassis power control
	*  V2.0.0     July-20-2022    Zelin Shen      2. re-write basic chassis power control function; support dynamic charging power
	*																								 add speed adaptive chassis power control;
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2022 RoboGrinder at Virginia Tech****************************
  */
#ifndef CHASSIS_POWER_CONTROL_H
#define CHASSIS_POWER_CONTROL_H
#include "chassis_task.h"
#include "main.h"

#define USE_SpeedAdaptiveChassisPowerControl 1
#define HERO_CHASSIS_POWER_CONTROL
//#define INFANTRY_CHASSIS_POWER_CONTROL

//底盘初始状态时 功率限制 也就是最低功率
#ifdef HERO_CHASSIS_POWER_CONTROL
#define INITIAL_STATE_CHASSIS_POWER_LIM 50.0f
#else
#define INITIAL_STATE_CHASSIS_POWER_LIM 40.0f
#endif

#define MAX_REASONABLE_CHARGE_PWR 160

/*
原始参数:
#define POWER_LIMIT         500.0f//500
#define WARNING_POWER       300.0f //40.0f   
#define WARNING_POWER_BUFF  50.0f   

WARNING_POWER需要小于POWER_LIMIT
5<chassis_power_buffer<(WARNING_POWER_BUFF=40)时, 只是*debuf => 做减缓处理 <=> buffer_debuff_total_current_limit * power_scale
buffer_debuff_total_current_limit 代表了这种情况下允许的 最大 功率(极限)

<5时, 保证不能再用缓冲能量了
7-10-2022 备份数值
#define POWER_LIMIT         500.0f//500
#define WARNING_POWER       300.0f//40.0f   
#define WARNING_POWER_BUFF  40.0f//50.0f   

#define NO_JUDGE_TOTAL_CURRENT_LIMIT    64000.0f    //16000 * 4, 
#define BUFFER_TOTAL_CURRENT_LIMIT      16000.0f
#define POWER_TOTAL_CURRENT_LIMIT       20000.0f
其它注释:
64000.0f = 16000 * 4, 16000指16A

无超级电容 直接连接
*/
#define POWER_LIMIT         500.0f//500
#define WARNING_POWER       400.0f//40.0f   
#define WARNING_POWER_BUFF  40.0f//50.0f   

#define NO_JUDGE_TOTAL_CURRENT_LIMIT    64000.0f   //裁判系统离线时总电流限制
#define BUFFER_TOTAL_CURRENT_LIMIT      40000.0f	//用于 当缓冲能量小于40j时 5<chassis_power_buffer<(WARNING_POWER_BUFF=40)
#define POWER_TOTAL_CURRENT_LIMIT       20000.0f  //功率限制 时 电流

#define POWER_TOTAL_CURRENT_LIMIT_WHEN_NO_BUFF_USED 20833.3f

//Speed-adaptive chassis power control; related parameter
/*调整了一次参数: 调整之前:
#define WARNING_ENERGY_BUFF  40.0f 
#define MINIMUM_ENERGY_BUFF 10.0f //设定缓冲能量 危险值
...
#define ENERGY_BUFF_OUTPUT_CUTOFF_POINT 3.0f
*/
#define WARNING_ENERGY_BUFF  35.0f //40.0f 
#define MINIMUM_ENERGY_BUFF  10.0f //10.0f //设定缓冲能量 危险值
#define MAX_POWER_VALUE 400.0f //300.0f//220.0f

#define REFEREE_OFFLINE_POWER_LIM_VAL MAX_POWER_VALUE //100.0f

/*When ENERGY_BUFF_OUTPUT_CUTOFF_POINT is reached, output will be disabled; and the buff eng need to be recharged to MINIMUM_ENERGY_BUFF to re-enable the output*/
#define ENERGY_BUFF_OUTPUT_CUTOFF_POINT 5.0f //3.0f

//裁判系统功率 缓冲能量信息 间隔; 实时功率热量数据：0x0202。发送频率：50Hz; 即0.02s
#define CHASSIS_REFEREE_COMM_TIME 0.02f;

/*
7-28-2022 之前参数:
#define superCap_WARNING_ENERGY_BUFF 1500.0f
#define superCap_MINIMUM_ENERGY_BUFF 700.0f
#define superCap_MAX_POWER_VALUE 300.0f

#define superCap_ENERGY_BUFF_OUTPUT_CUTOFF_POINT 650.0f

7-21-2022 用超级电容的功率控制; speed-adaptive chassis power control
*/
#define superCap_WARNING_ENERGY_BUFF 1200.0f //1500.0f
#define superCap_MINIMUM_ENERGY_BUFF 750.0f //700.0f
//7-28-2022晚, 由基于能量的调控改为 基于电压的调控, 因为这样要直观一点, 而且和 直连速度自适应底盘功率控制 算法中 数量数量级差不多
#define superCap_WARNING_VOL 20.0f //对应superCap_WARNING_ENERGY_BUFF
#define superCap_MINIMUM_VOL 15.81f //superCap_MINIMUM_ENERGY_BUFF
#define superCap_MAX_POWER_VALUE 400.0f//300.0f

#define superCap_REFEREE_OFFLINE_POWER_LIM_VAL superCap_MAX_POWER_VALUE

/*When superCap_ENERGY_BUFF_OUTPUT_CUTOFF_POINT is reached, output will be disabled; and the buff eng need to be recharged to MINIMUM_ENERGY_BUFF to re-enable the output*/
#define superCap_ENERGY_BUFF_OUTPUT_CUTOFF_POINT 650.0f//700.0f //650.0f
#define superCap_VOL_OUTPUT_CUTOFF_POINT 3.5f //13.50f //14.72f

//裁判系统功率 缓冲能量信息 间隔; 实时功率热量数据：0x0202。发送频率：50Hz; 即0.02s
#define superCap_CHASSIS_SUPERCAP_COMM_TIME 0.02f; //由于使用此数值的算法不是特别合理, 目前暂时就用0.02f

/*以下参数未使用*/
#define superCap_POWER_LIMIT         500.0f//500
#define superCap_WARNING_POWER       300.0f//40.0f   
#define superCap_WARNING_POWER_BUFF  40.0f//50.0f   

#define superCap_NO_JUDGE_TOTAL_CURRENT_LIMIT    64000.0f    //16000 * 4
#define superCap_BUFFER_TOTAL_CURRENT_LIMIT      16000.0f
#define superCap_POWER_TOTAL_CURRENT_LIMIT       20000.0f
/*--------------*/

typedef enum
{
	below_ENERGY_CRITICAL_POINT,
	above_ENERGY_CRITICAL_POINT,
}energy_buff_output_cutoff_point_status_e;//this enum is for debug only

typedef enum
{
	adp_cpc_MAX_loop_cnt_reached,
	adp_cpc_NORMAL,
}speed_adaptive_chassis_power_control_result_status_e;//这两个结构体 不同的超级电容共用

typedef struct
{
	 fp32 motor_final_current[4];
	 fp32 motor_final_total_current;
	 uint16_t chassis_power_limit;//当前机器人充电功率上限
	 uint16_t superCap_charge_pwr;//当前 超级电容 充电功率
	
	 fp32 chassis_power;//裁判系统 底盘功率, 这里代表了超级电容充电功率
   fp32 chassis_power_buffer;//裁判系统 60J缓冲能量
   fp32 superCap_e_buffer;
	 fp32 superCap_vol;
	
   fp32 total_current;
	
	 fp32 total_current_limit;
	
	 fp32 buffer_debuff_total_current_limit;
	
	 //...其它的不用 只需要速度自适应相关的变量
	 uint8_t robot_id;
	 
	 //speed adaptive related variable
	 speed_adaptive_chassis_power_control_result_status_e adp_pwr_ctrl_result_status;
	 fp32 total_current_unit_amp;
	
	 energy_buff_output_cutoff_point_status_e ene_cutoff_sts;
	 fp32 critical_val; //critical val to disable or enable the output; below this val=cutoff output; above this val=re-enable output
	
	 fp32 p_max;//当前帧算出来的 可用功率上限; 单位为 瓦
	 uint32_t max_speed_adp_loop_cnt; //max speed adaptive loop count when function runs once excluding max loop cnt; 最多的时候循环了几次
	 uint32_t num_of_normal_loop; //number of times that the speed adaptive mechanism worked normally
	 uint32_t num_loop_limit_reached; //number of times that the program reached the upper limit of the loop
	 uint32_t current_loop_cnt;
	
}chassis_energy_control_t;// 使用超级电容时的功率闭环结构体

//不使用超级电容时 直连时 的功率闭环结构体
typedef struct
{
	 fp32 motor_final_current[4];
	 fp32 motor_final_total_current;
	 uint16_t chassis_power_limit;//当前机器人充电功率上限
	 fp32 chassis_power;
   fp32 chassis_power_buffer;
   
   fp32 total_current;
	
	 fp32 total_current_limit;//=下面几个中的任意一个
	
	 fp32 buffer_debuff_total_current_limit; //用于 当缓冲能量小于40j时 5<chassis_power_buffer<(WARNING_POWER_BUFF=40) 处于减缓状态时的 总限制电流
	 fp32 buffer_minimum_total_current_limit; // <5 时的保证, 不会再用buffer能量了
	 fp32 power_total_current_limit; //功率 峰值限制 时 电流
	
	 uint8_t robot_id;
	 
	 //speed adaptive related variable
	 speed_adaptive_chassis_power_control_result_status_e adp_pwr_ctrl_result_status;
	 fp32 total_current_unit_amp;
	
	 energy_buff_output_cutoff_point_status_e ene_cutoff_sts;
	 fp32 critical_power_buffer; //critical val to disable or enable the output; below this val=cutoff output; above this val=re-enable output
	
	 fp32 p_max;//当前帧算出来的 可用功率上限; 单位为 瓦
	 uint32_t max_speed_adp_loop_cnt; //max speed adaptive loop count when function runs once excluding max loop cnt; 最多的时候循环了几次
	 uint32_t num_of_normal_loop; //number of times that the speed adaptive mechanism worked normally
	 uint32_t num_loop_limit_reached; //number of times that the program reached the upper limit of the loop
	 uint32_t current_loop_cnt;
}chassis_energy_control_direct_connect_t;

/*
 fp32 chassis_power = 0.0f;
 fp32 chassis_power_buffer = 0.0f;
 fp32 total_current_limit = 0.0f;
 fp32 total_current = 0.0f;
 robot_id = get_robot_id();
*/

/**
  * @brief          limit the power, mainly limit motor current
  * @param[in]      chassis_power_control: chassis data 
  * @retval         none
  */
/**
  * @brief          限制功率，主要限制电机电流
  * @param[in]      chassis_power_control: 底盘数据
  * @retval         none
  */
extern void chassis_power_control(chassis_move_t *chassis_power_control);
extern void speed_adaptive_chassis_power_control(chassis_move_t *chassis_power_control);
extern void superCap_speed_adaptive_chassis_power_control(chassis_move_t *chassis_power_control);
#endif
