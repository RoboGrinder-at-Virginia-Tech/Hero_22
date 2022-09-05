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
#include "chassis_power_control.h"
#include "referee.h"
#include "arm_math.h"
#include "detect_task.h"
#include "SuperCap_comm.h"

extern  supercap_can_msg_id_e current_superCap;

static void update_energy_buffer_debuff_total_current_limit(uint16_t chassis_p_lim, fp32* total_i_lim);

//调试用
uint8_t SZL_debug_place=4;
fp32 SZL_debug_chassis_power_buffer = 0;
//static uint8_t robot_id = 0 ; //test
//调试用END----

chassis_energy_control_t chassis_energy_control;
chassis_energy_control_direct_connect_t direct_connect_e_cont; //chassis_energy_control_direct_connect;

//regular power control global var - for debug
fp32 current_scale;

//speed adaptive power control global var - for debug
fp32 speed_adp_scale;

//void superCap_chassis_power_control(chassis_move_t *chassis_power_control)
//{
//	  //超级电容功率限制
//	  if(toe_is_error(SUPERCAP_TOE) && toe_is_error(WULIE_CAP_TOE))
//		{
//			  total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
//		}
//	  else //if(current_superCap == SuperCap_ID)
//	  {
//			  if(current_superCap == SuperCap_ID)
//				{
//					chassis_energy_control.current_superCap_buffer = 
//				}
//				else
//				{
//					
//				}
//			  get_chassis_power_and_buffer(&chassis_energy_control.chassis_power, &chassis_energy_control.chassis_power_buffer);
//	  }
////		else if(current_superCap == wulie_Cap_CAN_ID)
////		{
////			  
////		}
//}

//调控速度; 速度自适应的 功率控制; 结合 分段常数控制器
void speed_adaptive_chassis_power_control(chassis_move_t *chassis_power_control)
{
	  //fp32 current_scale;
	
		direct_connect_e_cont.robot_id = get_robot_id();
	
		/*---更新一下需要用到的 动态变动的数据---*/
	  direct_connect_e_cont.chassis_power_limit = get_chassis_power_limit();
	  if(direct_connect_e_cont.chassis_power_limit > 101)
		{//识别 并处理 不合理数值
			direct_connect_e_cont.chassis_power_limit = 50;
		}
		
		//从裁判系统获取当前缓冲能量
		get_chassis_power_and_buffer(&direct_connect_e_cont.chassis_power, &direct_connect_e_cont.chassis_power_buffer);
		
		//识别 并处理 chassis_power 和 chassis_power_buffer 不合理数值；--- SZL: 暂时不处理 ---
		
		//judge output cut-off point based on remaining energy and set the buffer ene critical val point
		/* ENERGY_BUFF_OUTPUT_CUTOFF_POINT = 3; MINIMUM_ENERGY_BUFF=10*/
		if(direct_connect_e_cont.chassis_power_buffer <= ENERGY_BUFF_OUTPUT_CUTOFF_POINT)
		{//一定产生cut off条件
			direct_connect_e_cont.critical_power_buffer = MINIMUM_ENERGY_BUFF;
		}
		else if(direct_connect_e_cont.chassis_power_buffer >= MINIMUM_ENERGY_BUFF)
		{//一定关闭cut off条件
			direct_connect_e_cont.critical_power_buffer = ENERGY_BUFF_OUTPUT_CUTOFF_POINT;
		}
		else
		{// default sts
			direct_connect_e_cont.critical_power_buffer = ENERGY_BUFF_OUTPUT_CUTOFF_POINT;
		}
		
		//分层 计算当前可用功率上限
		if(direct_connect_e_cont.chassis_power_buffer >= WARNING_ENERGY_BUFF)
		{//功率限制
//			direct_connect_e_cont.p_max = (fp32)(direct_connect_e_cont.chassis_power_buffer - MINIMUM_ENERGY_BUFF) / CHASSIS_REFEREE_COMM_TIME;
			direct_connect_e_cont.p_max = MAX_POWER_VALUE;
			
			direct_connect_e_cont.p_max = fp32_constrain(direct_connect_e_cont.p_max, INITIAL_STATE_CHASSIS_POWER_LIM, MAX_POWER_VALUE);//最大功率的 限制
			//convert p_max to total_current_limit for esc raw values
		  direct_connect_e_cont.total_current_limit = (fp32)direct_connect_e_cont.p_max / 24.0f * 1000.0f;//* 1000.0f is to convert metric unit var to esc control raw value
		}
		else if(direct_connect_e_cont.chassis_power_buffer > MINIMUM_ENERGY_BUFF && direct_connect_e_cont.chassis_power_buffer < WARNING_ENERGY_BUFF)
		{//直接电流限制; 这样比较方便; 减缓
			fp32 power_scale = direct_connect_e_cont.chassis_power_buffer / WARNING_ENERGY_BUFF;
//			update_energy_buffer_debuff_total_current_limit(direct_connect_e_cont.chassis_power_limit, &direct_connect_e_cont.buffer_debuff_total_current_limit);
//			direct_connect_e_cont.total_current_limit = direct_connect_e_cont.buffer_debuff_total_current_limit * power_scale;
			direct_connect_e_cont.total_current_limit = 16000.0f * power_scale;
			
			//反着更新 p_max
			direct_connect_e_cont.p_max = direct_connect_e_cont.total_current_limit / 1000.0f * 24.0f;
		}
		else
		{//功率限制
			//缓冲能量达到或者小于危险值了, 保证当前底盘输出功率 小于等于 裁判系统的功率上限
			direct_connect_e_cont.p_max = (fp32)direct_connect_e_cont.chassis_power_limit;//-8.0f;
			
			direct_connect_e_cont.p_max = fp32_constrain(direct_connect_e_cont.p_max, INITIAL_STATE_CHASSIS_POWER_LIM, MAX_POWER_VALUE);//最大功率的 限制
			//convert p_max to total_current_limit for esc raw values
		  direct_connect_e_cont.total_current_limit = (fp32)direct_connect_e_cont.p_max / 24.0f * 1000.0f;//* 1000.0f is to convert metric unit var to esc control raw value
		}
		
//		//在这里实现 最大功率的 限制
//		if(fabs(direct_connect_e_cont.p_max) > MAX_POWER_VALUE)
//		{
//			direct_connect_e_cont.p_max = MAX_POWER_VALUE;
//		}
		
		/*---完成 动态变动的数据 的更新---*/
		
		/*先处理 裁判系统离线的情况---就只限制输出功率*/
		if(toe_is_error(REFEREE_TOE))
		{
			//就按找一个功率来限制就行了; 设备离线; 特殊情况下 的数据更新
			direct_connect_e_cont.p_max = REFEREE_OFFLINE_POWER_LIM_VAL;
			direct_connect_e_cont.p_max = fp32_constrain(direct_connect_e_cont.p_max, INITIAL_STATE_CHASSIS_POWER_LIM, REFEREE_OFFLINE_POWER_LIM_VAL);
			direct_connect_e_cont.total_current_limit = (fp32)direct_connect_e_cont.p_max / 24.0f * 1000.0f;
			
			//calculate pid
			/*调试时发现的一个现象: PID算法; set=0; fdb=0; error=0; 由于I项, Iout=922; out=530.809
			  即total_current=1500~3000时 底盘 极低功率; 
			  测试数据: 裁判系统离线时如果 p_max = 100 -> total_current_limit=2083.3; total_current = 3630.90; 操作界面显示的chassis_power = 3.5w;
			
				(已用 分段常数控制器 解决)把机器人架起来, 摇杆向前推, 轮子向前空转到最大速度后; PID set 接近 fdb; 使得 out不高
			  即低total_current=1500~3000时 较高底盘功率出现;
				测试数据: 摇杆推到最前面; total_current = 1295.1; total_current_limit=20833.3(p_max=499.9); 数据包接收到的 chassis_power = 49.43w; 使用功率计测到的功率也差不多
				
				------ 第一个问题怎么解决呢? ------ 目前暂时把REFEREE_OFFLINE_POWER_LIM_VAL设高
			*/
			for (uint8_t i = 0; i < 4; i++)
			{
					PID_calc(&chassis_power_control->motor_speed_pid[i], chassis_power_control->motor_chassis[i].speed, chassis_power_control->motor_chassis[i].speed_set);
			}
				
			direct_connect_e_cont.total_current = 0.0f;
			//calculate the original motor current set
			//计算原本电机电流设定
			for(uint8_t i = 0; i < 4; i++)
			{
					direct_connect_e_cont.total_current += fabs(chassis_power_control->motor_speed_pid[i].out);
			}
			
			if(direct_connect_e_cont.total_current > direct_connect_e_cont.total_current_limit)
			{
				current_scale = direct_connect_e_cont.total_current_limit / direct_connect_e_cont.total_current;
				chassis_power_control->motor_speed_pid[0].out*=current_scale;
				chassis_power_control->motor_speed_pid[1].out*=current_scale;
				chassis_power_control->motor_speed_pid[2].out*=current_scale;
				chassis_power_control->motor_speed_pid[3].out*=current_scale;
			}
		}/*开始 分段常数控制器 + 速度自适应的功率控制*/
		else if(direct_connect_e_cont.chassis_power_buffer < direct_connect_e_cont.critical_power_buffer)
		{//when below critical pt; just cut-off output
			chassis_power_control->motor_speed_pid[0].out = 0.0f;
			chassis_power_control->motor_speed_pid[1].out = 0.0f;
			chassis_power_control->motor_speed_pid[2].out = 0.0f;
			chassis_power_control->motor_speed_pid[3].out = 0.0f;
			
			direct_connect_e_cont.ene_cutoff_sts = below_ENERGY_CRITICAL_POINT;//for debug
		}
		else
		{
			direct_connect_e_cont.ene_cutoff_sts = above_ENERGY_CRITICAL_POINT;//for debug
			
			direct_connect_e_cont.current_loop_cnt = 0;// init value
			while(1)
			{
				//calculate pid
				for (uint8_t i = 0; i < 4; i++)
				{
						PID_calc(&chassis_power_control->motor_speed_pid[i], chassis_power_control->motor_chassis[i].speed, chassis_power_control->motor_chassis[i].speed_set);
				}
				
				direct_connect_e_cont.total_current = 0.0f;
				//calculate the original motor current set
				//计算原本电机电流设定
				for(uint8_t i = 0; i < 4; i++)
				{
						direct_connect_e_cont.total_current += fabs(chassis_power_control->motor_speed_pid[i].out);
				}
				direct_connect_e_cont.total_current_unit_amp = direct_connect_e_cont.total_current / 1000.0f;//convert esc control value to unit amp current
				
				if(direct_connect_e_cont.total_current > direct_connect_e_cont.total_current_limit)//direct_connect_e_cont.total_current_unit_amp * 24.0f > direct_connect_e_cont.p_max)
				{
	//				  fp32 speed_adp_scale;
						
						direct_connect_e_cont.current_loop_cnt++;
						if(direct_connect_e_cont.current_loop_cnt >= 8)
						{
							//达到设定循环次数上限 直接削弱目标电流来保证
							//direct_connect_e_cont.total_current_limit = (fp32)direct_connect_e_cont.p_max / 24.0f * 1000.0f;//* 1000.0f is to convert metric unit var to esc control raw value
	//						if(direct_connect_e_cont.total_current > direct_connect_e_cont.total_current_limit)
	//						{
								//fp32 current_scale = direct_connect_e_cont.total_current_limit / direct_connect_e_cont.total_current;
								current_scale = direct_connect_e_cont.total_current_limit / direct_connect_e_cont.total_current;
								chassis_power_control->motor_speed_pid[0].out*=current_scale;
								chassis_power_control->motor_speed_pid[1].out*=current_scale;
								chassis_power_control->motor_speed_pid[2].out*=current_scale;
								chassis_power_control->motor_speed_pid[3].out*=current_scale;
	//						}
							direct_connect_e_cont.adp_pwr_ctrl_result_status = adp_cpc_MAX_loop_cnt_reached;
							break;
						}
						else
						{
							//adapt speed
							speed_adp_scale = 0.99f; //direct_connect_e_cont.total_current_limit / direct_connect_e_cont.total_current; //direct_connect_e_cont.p_max / (direct_connect_e_cont.total_current_unit_amp * 24.0f);
							chassis_power_control->motor_chassis[0].speed_set *= speed_adp_scale;
							chassis_power_control->motor_chassis[1].speed_set *= speed_adp_scale;
							chassis_power_control->motor_chassis[2].speed_set *= speed_adp_scale;
							chassis_power_control->motor_chassis[3].speed_set *= speed_adp_scale;
						}
				}
				else
				{
					direct_connect_e_cont.adp_pwr_ctrl_result_status = adp_cpc_NORMAL;
					break;
				}
			}
		}
		
		//values and FSM for debug regarding speed-adaptive power ctrl algorithm
		if(direct_connect_e_cont.adp_pwr_ctrl_result_status == adp_cpc_MAX_loop_cnt_reached)
		{
			direct_connect_e_cont.num_loop_limit_reached++;
		}
		else
		{
			if(direct_connect_e_cont.current_loop_cnt != 0)
			{
				direct_connect_e_cont.num_of_normal_loop++;
			}
			
			if(direct_connect_e_cont.current_loop_cnt > direct_connect_e_cont.max_speed_adp_loop_cnt)
			{
				direct_connect_e_cont.max_speed_adp_loop_cnt = direct_connect_e_cont.current_loop_cnt;
			}
		}
		
		//values for debug
		direct_connect_e_cont.motor_final_current[0] = chassis_power_control->motor_speed_pid[0].out;
		direct_connect_e_cont.motor_final_current[1] = chassis_power_control->motor_speed_pid[1].out;
		direct_connect_e_cont.motor_final_current[2] = chassis_power_control->motor_speed_pid[2].out;
		direct_connect_e_cont.motor_final_current[3] = chassis_power_control->motor_speed_pid[3].out;
		
		direct_connect_e_cont.motor_final_total_current = 0;
		for(uint8_t i = 0; i < 4; i++)
		{
			direct_connect_e_cont.motor_final_total_current += fabs(direct_connect_e_cont.motor_final_current[i]);
		}
		
}

static void update_energy_buffer_debuff_total_current_limit(uint16_t chassis_p_lim, fp32* total_i_lim)
{
#ifdef HERO_CHASSIS_POWER_CONTROL
		/* Hero 通过标定 将功率上限 映射为 电机最大电流值; 对于debuff这一档 */
		//功率优先
		if(chassis_p_lim == 50)
		{
			*total_i_lim = 16000.0f;
		}
		else if(chassis_p_lim == 70)
		{
			*total_i_lim = 16000.0f;
		}
		else if(chassis_p_lim == 90)
		{
			*total_i_lim = 16000.0f;
		}
		else if(chassis_p_lim == 120)
		{
			*total_i_lim = 16000.0f;
		}
		else if(chassis_p_lim == 55) //血量优先
		{
			*total_i_lim = 16000.0f;
		}
		else if(chassis_p_lim == 60)
		{
			*total_i_lim = 16000.0f;
		}
		else if(chassis_p_lim == 65)
		{
			*total_i_lim = 16000.0f;
		}
		else
		{
			*total_i_lim = 16000.0f;
		}
#else
		/*Infantry 通过标定 将功率上限 映射为 电机最大电流值; 对于debuff这一档 */
		//功率优先
		if(chassis_p_lim == 40)
		{
			*total_i_lim = 16000.0f;
		}
		else if(chassis_p_lim == 60)
		{
			*total_i_lim = 16000.0f;
		}
		else if(chassis_p_lim == 80)
		{
			*total_i_lim = 16000.0f;
		}
		else if(chassis_p_lim == 100)
		{
			*total_i_lim = 16000.0f;
		}
		else if(chassis_p_lim == 45) //血量优先
		{
			*total_i_lim = 16000.0f;
		}
		else if(chassis_p_lim == 50)
		{
			*total_i_lim = 16000.0f;
		}
		else if(chassis_p_lim == 55)
		{
			*total_i_lim = 16000.0f;
		}
		else
		{
			*total_i_lim = 16000.0f;
		}
#endif
}

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
void chassis_power_control(chassis_move_t *chassis_power_control)
{//非超级电容; 直连 功率闭环
//    fp32 chassis_power = 0.0f;
//    fp32 chassis_power_buffer = 0.0f;
//    fp32 total_current_limit = 0.0f;
//    fp32 total_current = 0.0f;
    direct_connect_e_cont.robot_id = get_robot_id();
		
	  /*--- 更新一下需要用到的 动态变动的数据 ---*/
	  direct_connect_e_cont.chassis_power_limit = get_chassis_power_limit();
	  if(direct_connect_e_cont.chassis_power_limit > 101)
		{//识别不合理数值
			direct_connect_e_cont.chassis_power_limit = 50;
		}
		//w=VI 将等级功率上限 映射为 电机最大电流值
		//功率优先
		if(direct_connect_e_cont.chassis_power_limit == 50)
		{
			direct_connect_e_cont.buffer_debuff_total_current_limit = 16000.0f;
			direct_connect_e_cont.buffer_minimum_total_current_limit = 2080.0f;
		}
		else if(direct_connect_e_cont.chassis_power_limit == 70)
		{
			direct_connect_e_cont.buffer_debuff_total_current_limit = 16000.0f;
			direct_connect_e_cont.buffer_minimum_total_current_limit = 2916.0f;
		}
		else if(direct_connect_e_cont.chassis_power_limit == 90)
		{
			direct_connect_e_cont.buffer_debuff_total_current_limit = 16000.0f;
			direct_connect_e_cont.buffer_minimum_total_current_limit = 3750.0f;
		}
		else if(direct_connect_e_cont.chassis_power_limit == 120)
		{
			direct_connect_e_cont.buffer_debuff_total_current_limit = 16000.0f;
			direct_connect_e_cont.buffer_minimum_total_current_limit = 5000.0f;
		}
		else if(direct_connect_e_cont.chassis_power_limit == 55) //血量优先
		{
			direct_connect_e_cont.buffer_debuff_total_current_limit = 16000.0f;
			direct_connect_e_cont.buffer_minimum_total_current_limit = 2291.6f;
		}
		else if(direct_connect_e_cont.chassis_power_limit == 60)
		{
			direct_connect_e_cont.buffer_debuff_total_current_limit = 16000.0f;
			direct_connect_e_cont.buffer_minimum_total_current_limit = 2500.0f;
		}
		else if(direct_connect_e_cont.chassis_power_limit == 65)
		{
			direct_connect_e_cont.buffer_debuff_total_current_limit = 16000.0f;
			direct_connect_e_cont.buffer_minimum_total_current_limit = 2708.0f;
		}
		else
		{
			 direct_connect_e_cont.chassis_power_limit = 50;
			 direct_connect_e_cont.buffer_debuff_total_current_limit = 16000.0f;
			 direct_connect_e_cont.buffer_minimum_total_current_limit = 2080.0f;
		}
		
		//从裁判系统获取当前缓冲能量
		get_chassis_power_and_buffer(&direct_connect_e_cont.chassis_power, &direct_connect_e_cont.chassis_power_buffer);
		
		//识别 并处理 chassis_power 和 chassis_power_buffer 不合理数值；--- SZL: 暂时不处理 ---
		
		//judge output cut-off point based on remaining energy and set the buffer ene critical val point
		/* ENERGY_BUFF_OUTPUT_CUTOFF_POINT = 3; MINIMUM_ENERGY_BUFF=10*/
		if(direct_connect_e_cont.chassis_power_buffer <= ENERGY_BUFF_OUTPUT_CUTOFF_POINT)
		{//一定产生cut off条件
			direct_connect_e_cont.critical_power_buffer = MINIMUM_ENERGY_BUFF;
		}
		else if(direct_connect_e_cont.chassis_power_buffer >= MINIMUM_ENERGY_BUFF)
		{//一定关闭cut off条件
			direct_connect_e_cont.critical_power_buffer = ENERGY_BUFF_OUTPUT_CUTOFF_POINT;
		}
		else
		{// default sts
			direct_connect_e_cont.critical_power_buffer = ENERGY_BUFF_OUTPUT_CUTOFF_POINT;
		}
		/*--- 完成动态数据的更新 ---*/
		
		
		/*开始 分段常数控制器 + 底盘功率控制*/
		if(direct_connect_e_cont.chassis_power_buffer < direct_connect_e_cont.critical_power_buffer)
		{//when below critical pt; just cut-off output
			chassis_power_control->motor_speed_pid[0].out = 0.0f;
			chassis_power_control->motor_speed_pid[1].out = 0.0f;
			chassis_power_control->motor_speed_pid[2].out = 0.0f;
			chassis_power_control->motor_speed_pid[3].out = 0.0f;
			
			direct_connect_e_cont.ene_cutoff_sts = below_ENERGY_CRITICAL_POINT;//for debug
		}
		else
		{
			if(toe_is_error(REFEREE_TOE))
			{
					direct_connect_e_cont.total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
			}
			else if(direct_connect_e_cont.robot_id == RED_ENGINEER || direct_connect_e_cont.robot_id == BLUE_ENGINEER || direct_connect_e_cont.robot_id == 0)
			{
					direct_connect_e_cont.total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
			}
			else
			{
	//        get_chassis_power_and_buffer(&direct_connect_e_cont.chassis_power, &direct_connect_e_cont.chassis_power_buffer);
					// power > 80w and buffer < 60j, because buffer < 60 means power has been more than 80w
					//功率超过80w 和缓冲能量小于60j,因为缓冲能量小于60意味着功率超过80w
					/* 举例 步兵机器人或英雄机器人未触发飞坡增益时，缓冲能量上限为 60J
						缓冲能量小于60j, 说明当前功率超过 当前的功率等级上限
						缓存能量剩50j时, 不管; 当缓冲能量小于40j时, 开始管 进入这个 if
					*/
					if(direct_connect_e_cont.chassis_power_buffer < WARNING_POWER_BUFF)
					{
							fp32 power_scale;
							if(direct_connect_e_cont.chassis_power_buffer > 10.0f)
							{
									//scale down WARNING_POWER_BUFF
									//缩小WARNING_POWER_BUFF
									//SZL: 10<chassis_power_buffer<(WARNING_POWER_BUFF=40)
									power_scale = direct_connect_e_cont.chassis_power_buffer / WARNING_POWER_BUFF;
									direct_connect_e_cont.total_current_limit = direct_connect_e_cont.buffer_debuff_total_current_limit * power_scale;
									SZL_debug_place = 1;
							}
							else
							{
									//only left 10% of WARNING_POWER_BUFF//小于5的时候都用5来限制幅度
									//power_scale = 5.0f / WARNING_POWER_BUFF;
									direct_connect_e_cont.total_current_limit = direct_connect_e_cont.buffer_minimum_total_current_limit;
									SZL_debug_place = 2;
							}
							/*scale down 缩小 SZL 7-15-2022 修改
								根据当前等级 允许的充电功率 来限制, 即<=> 当前充电功率所对应的电机电流数值 * power_scale
							*/
							//direct_connect_e_cont.total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT * power_scale;
							//SZL_debug_chassis_power_buffer = chassis_power_buffer;//SZL 后来加的
					}
					else
					{
							/*power > WARNING_POWER 功率大于WARNING_POWER; WARNING_POWER=400; POWER_LIMIT=500
								
							*/
							if(direct_connect_e_cont.chassis_power > WARNING_POWER)
							{
									fp32 power_scale;
									if(direct_connect_e_cont.chassis_power < POWER_LIMIT)
									{
											/*scale down;
												WARNING_POWER=400 < chassis_power < POWER_LIMIT=500
											*/
											power_scale = (POWER_LIMIT - direct_connect_e_cont.chassis_power) / (POWER_LIMIT - WARNING_POWER);
											SZL_debug_place = 3;
									}
									else
									{
											//chassis_power > POWER_LIMIT=500
											power_scale = 0.0f;
											SZL_debug_place = 4;
									}
									//total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT * power_scale;
									direct_connect_e_cont.total_current_limit = POWER_TOTAL_CURRENT_LIMIT_WHEN_NO_BUFF_USED * power_scale;
							}
							//power < WARNING_POWER
							//功率小于WARNING_POWER
							else
							{
									//total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT;
									direct_connect_e_cont.total_current_limit = POWER_TOTAL_CURRENT_LIMIT_WHEN_NO_BUFF_USED;
									SZL_debug_place = 5;
							}
	//              direct_connect_e_cont.total_current_limit = 64000.0f; //POWER_TOTAL_CURRENT_LIMIT_WHEN_NO_BUFF_USED;
					}
			}

			
			direct_connect_e_cont.total_current = 0.0f;
			//calculate the original motor current set
			//计算原本电机电流设定
			for(uint8_t i = 0; i < 4; i++)
			{
					direct_connect_e_cont.total_current += fabs(chassis_power_control->motor_speed_pid[i].out);
			}
			

			if(direct_connect_e_cont.total_current > direct_connect_e_cont.total_current_limit)
			{
					//fp32 current_scale = direct_connect_e_cont.total_current_limit / direct_connect_e_cont.total_current;
					current_scale = direct_connect_e_cont.total_current_limit / direct_connect_e_cont.total_current;
					chassis_power_control->motor_speed_pid[0].out*=current_scale;
					chassis_power_control->motor_speed_pid[1].out*=current_scale;
					chassis_power_control->motor_speed_pid[2].out*=current_scale;
					chassis_power_control->motor_speed_pid[3].out*=current_scale;
			}
		}
		
		//values for debug
		direct_connect_e_cont.motor_final_current[0] = chassis_power_control->motor_speed_pid[0].out;
		direct_connect_e_cont.motor_final_current[1] = chassis_power_control->motor_speed_pid[1].out;
		direct_connect_e_cont.motor_final_current[2] = chassis_power_control->motor_speed_pid[2].out;
		direct_connect_e_cont.motor_final_current[3] = chassis_power_control->motor_speed_pid[3].out;
		
		direct_connect_e_cont.motor_final_total_current = 0;
		for(uint8_t i = 0; i < 4; i++)
		{
				direct_connect_e_cont.motor_final_total_current += fabs(direct_connect_e_cont.motor_final_current[i]);
		}

}
