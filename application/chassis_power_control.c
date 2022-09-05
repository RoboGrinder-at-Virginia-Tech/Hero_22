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
	*  V3.0.0     August-4-2022   Zelin Shen      3. add chassis power & energy control using superCap; It is a speed adaptive chassis power control
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
static void map_superCap_charge_pwr_to_debuff_total_current_limit(uint16_t charge_pwr, fp32* total_i_lim);

//调试用
uint8_t SZL_debug_place=4;
fp32 SZL_debug_chassis_power_buffer = 0;
//static uint8_t robot_id = 0 ; //test
//调试用END----

chassis_energy_control_t chassis_e_ctrl;
chassis_energy_control_direct_connect_t direct_connect_e_cont; //chassis_energy_control_direct_connect;

//regular power control global var - for debug
fp32 current_scale;

//speed adaptive power control global var - for debug
fp32 speed_adp_scale;

void superCap_speed_adaptive_chassis_power_control(chassis_move_t *chassis_power_control)
{
		chassis_e_ctrl.robot_id = get_robot_id();
	
		/*---更新一下需要用到的 动态变动的数据---*/
	  chassis_e_ctrl.chassis_power_limit = get_chassis_power_limit();
		if(chassis_e_ctrl.chassis_power_limit>MAX_REASONABLE_CHARGE_PWR) //( (chassis_e_ctrl.chassis_power_limit>MAX_REASONABLE_CHARGE_PWR) || (chassis_e_ctrl.chassis_power_limit <0) )
		{//识别 并处理 不合理数值
			chassis_e_ctrl.chassis_power_limit = INITIAL_STATE_CHASSIS_POWER_LIM;
		}
		
		//从裁判系统获取当前缓冲能量
		get_chassis_power_and_buffer(&chassis_e_ctrl.chassis_power, &chassis_e_ctrl.chassis_power_buffer);//不重要
		//识别 并处理 chassis_power 和 chassis_power_buffer 不合理数值；--- SZL: 暂时不处理 --- 且这下数据对有超级电容来说 不是很重要
		
		//从 超级电容 获取当前剩余能量 获取当前使用的超级电容的剩余能量
	  get_superCap_vol_and_energy(&chassis_e_ctrl.superCap_vol, &chassis_e_ctrl.superCap_e_buffer);
		chassis_e_ctrl.superCap_charge_pwr = (uint16_t)get_superCap_charge_pwr();
		
		//judge output cut-off point based on remaining energy and set the buffer ene critical val point
		/* VOL_OUTPUT_CUTOFF_POINT = 14.72f; MINIMUM_VOL=15.81f*/
		if(chassis_e_ctrl.superCap_vol <= superCap_VOL_OUTPUT_CUTOFF_POINT) //superCap_ENERGY_BUFF_OUTPUT_CUTOFF_POINT)
		{//一定产生cut off条件
			chassis_e_ctrl.critical_val = superCap_MINIMUM_VOL; //superCap_MINIMUM_ENERGY_BUFF;
		}
		else if(chassis_e_ctrl.superCap_vol >= superCap_MINIMUM_VOL)
		{//一定关闭cut off条件
			chassis_e_ctrl.critical_val = superCap_VOL_OUTPUT_CUTOFF_POINT; //superCap_ENERGY_BUFF_OUTPUT_CUTOFF_POINT;
		}
		else
		{// default sts
			chassis_e_ctrl.critical_val = superCap_VOL_OUTPUT_CUTOFF_POINT;
		}
		
		//分层 计算当前可用功率上限
		if(chassis_e_ctrl.superCap_vol >= superCap_WARNING_VOL)
		{//功率限制
//			direct_connect_e_cont.p_max = (fp32)(direct_connect_e_cont.chassis_power_buffer - MINIMUM_ENERGY_BUFF) / CHASSIS_REFEREE_COMM_TIME;
			chassis_e_ctrl.p_max = superCap_MAX_POWER_VALUE;
			
			chassis_e_ctrl.p_max = fp32_constrain(chassis_e_ctrl.p_max, INITIAL_STATE_CHASSIS_POWER_LIM, superCap_MAX_POWER_VALUE);//最大功率的 限制
			//convert p_max to total_current_limit for esc raw values
		  chassis_e_ctrl.total_current_limit = (fp32)chassis_e_ctrl.p_max / 24.0f * 1000.0f;//* 1000.0f is to convert metric unit var to esc control raw value
		}
		else if(chassis_e_ctrl.superCap_vol > superCap_MINIMUM_VOL && chassis_e_ctrl.superCap_vol < superCap_WARNING_VOL)
		{//直接电流限制; 这样比较方便; 减缓
			/* 为了有更加平滑的能量控制 通过当前超级电容的充电功率 来映射出 当前debuff的最大功率
			*/
//			chassis_e_ctrl.p_max = 200.0f; //(fp32)(0.5f*6.0f*chassis_e_ctrl.superCap_vol*chassis_e_ctrl.superCap_vol - 0.5f*6.0f*superCap_WARNING_VOL*superCap_WARNING_VOL) / CHASSIS_REFEREE_COMM_TIME;
//			chassis_e_ctrl.p_max = fp32_constrain(chassis_e_ctrl.p_max, INITIAL_STATE_CHASSIS_POWER_LIM, superCap_MAX_POWER_VALUE);
//			
////			fp32 power_scale = chassis_e_ctrl.superCap_vol / superCap_WARNING_VOL;
//			
//			chassis_e_ctrl.total_current_limit = (fp32)chassis_e_ctrl.p_max / 24.0f * 1000.0f;// * power_scale;
//			//反着更新 p_max
//			chassis_e_ctrl.p_max = chassis_e_ctrl.total_current_limit / 1000.0f * 24.0f;
			//-------------------------------------------
			
//			fp32 power_scale = chassis_e_ctrl.superCap_vol / superCap_WARNING_VOL;
////			update_energy_buffer_debuff_total_current_limit(direct_connect_e_cont.chassis_power_limit, &direct_connect_e_cont.buffer_debuff_total_current_limit);
////			direct_connect_e_cont.total_current_limit = direct_connect_e_cont.buffer_debuff_total_current_limit * power_scale;
//			chassis_e_ctrl.total_current_limit = 16000.0f * power_scale; //16000.0f * power_scale;
//			
//			//反着更新 p_max
//			chassis_e_ctrl.p_max = chassis_e_ctrl.total_current_limit / 1000.0f * 24.0f;
			//8-4-2022 新方法---------------------------------------------------------------------------------------------------
			fp32 power_scale = chassis_e_ctrl.superCap_vol / superCap_WARNING_VOL;
			map_superCap_charge_pwr_to_debuff_total_current_limit(chassis_e_ctrl.superCap_charge_pwr, &(chassis_e_ctrl.buffer_debuff_total_current_limit));//fp32 buffer_debuff_total_current_limit;
			chassis_e_ctrl.total_current_limit = chassis_e_ctrl.buffer_debuff_total_current_limit * power_scale;
			//反着更新 p_max
			chassis_e_ctrl.p_max = chassis_e_ctrl.total_current_limit / 1000.0f * 24.0f;
		}
		else
		{//功率限制
			//缓冲能量达到或者小于危险值了, 保证当前底盘输出功率 小于等于 裁判系统的功率上限
			chassis_e_ctrl.p_max = (fp32)chassis_e_ctrl.superCap_charge_pwr - 4.0f; //(fp32)chassis_e_ctrl.chassis_power_limit - 4.0f;
			
			chassis_e_ctrl.p_max = fp32_constrain(chassis_e_ctrl.p_max, INITIAL_STATE_CHASSIS_POWER_LIM, superCap_MAX_POWER_VALUE);//最大功率的 限制
			//convert p_max to total_current_limit for esc raw values
		  chassis_e_ctrl.total_current_limit = (fp32)chassis_e_ctrl.p_max / 24.0f * 1000.0f;//* 1000.0f is to convert metric unit var to esc control raw value
		}
		
//		//在这里实现 最大功率的 限制
//		if(fabs(direct_connect_e_cont.p_max) > MAX_POWER_VALUE)
//		{
//			direct_connect_e_cont.p_max = MAX_POWER_VALUE;
//		}
		
		/*---完成 动态变动的数据 的更新---*/
		
		/*先处理 裁判系统离线的情况---就只限制输出功率*/
		if(current_superCap_is_offline())
		{
			//就按找一个功率来限制就行了; 设备离线; 特殊情况下 的数据更新
			chassis_e_ctrl.p_max = REFEREE_OFFLINE_POWER_LIM_VAL;
			chassis_e_ctrl.p_max = fp32_constrain(chassis_e_ctrl.p_max, INITIAL_STATE_CHASSIS_POWER_LIM, REFEREE_OFFLINE_POWER_LIM_VAL);
			chassis_e_ctrl.total_current_limit = (fp32)chassis_e_ctrl.p_max / 24.0f * 1000.0f;
			
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
				
			chassis_e_ctrl.total_current = 0.0f;
			//calculate the original motor current set
			//计算原本电机电流设定
			for(uint8_t i = 0; i < 4; i++)
			{
					chassis_e_ctrl.total_current += fabs(chassis_power_control->motor_speed_pid[i].out);
			}
			
			if(chassis_e_ctrl.total_current > chassis_e_ctrl.total_current_limit)
			{
				current_scale = chassis_e_ctrl.total_current_limit / chassis_e_ctrl.total_current;
				chassis_power_control->motor_speed_pid[0].out*=current_scale;
				chassis_power_control->motor_speed_pid[1].out*=current_scale;
				chassis_power_control->motor_speed_pid[2].out*=current_scale;
				chassis_power_control->motor_speed_pid[3].out*=current_scale;
			}
		}/*开始 分段常数控制器 + 速度自适应的功率控制*/
		else if(chassis_e_ctrl.superCap_vol < chassis_e_ctrl.critical_val)
		{//when below critical pt; just cut-off output
			chassis_power_control->motor_speed_pid[0].out = 0.0f;
			chassis_power_control->motor_speed_pid[1].out = 0.0f;
			chassis_power_control->motor_speed_pid[2].out = 0.0f;
			chassis_power_control->motor_speed_pid[3].out = 0.0f;
			
			chassis_e_ctrl.ene_cutoff_sts = below_ENERGY_CRITICAL_POINT;//for debug
		}
		else
		{
			chassis_e_ctrl.ene_cutoff_sts = above_ENERGY_CRITICAL_POINT;//for debug
			
			chassis_e_ctrl.current_loop_cnt = 0;// init value
			while(1)
			{
				//calculate pid
				for (uint8_t i = 0; i < 4; i++)
				{
						PID_calc(&chassis_power_control->motor_speed_pid[i], chassis_power_control->motor_chassis[i].speed, chassis_power_control->motor_chassis[i].speed_set);
				}
				
				chassis_e_ctrl.total_current = 0.0f;
				//calculate the original motor current set
				//计算原本电机电流设定
				for(uint8_t i = 0; i < 4; i++)
				{
						chassis_e_ctrl.total_current += fabs(chassis_power_control->motor_speed_pid[i].out);
				}
				chassis_e_ctrl.total_current_unit_amp = chassis_e_ctrl.total_current / 1000.0f;//convert esc control value to unit amp current
				
				if(chassis_e_ctrl.total_current > chassis_e_ctrl.total_current_limit)//direct_connect_e_cont.total_current_unit_amp * 24.0f > direct_connect_e_cont.p_max)
				{
	//				  fp32 speed_adp_scale;
						
						chassis_e_ctrl.current_loop_cnt++;
						if(chassis_e_ctrl.current_loop_cnt >= 8)
						{
							//达到设定循环次数上限 直接削弱目标电流来保证
							//direct_connect_e_cont.total_current_limit = (fp32)direct_connect_e_cont.p_max / 24.0f * 1000.0f;//* 1000.0f is to convert metric unit var to esc control raw value
	//						if(direct_connect_e_cont.total_current > direct_connect_e_cont.total_current_limit)
	//						{
								//fp32 current_scale = direct_connect_e_cont.total_current_limit / direct_connect_e_cont.total_current;
								current_scale = chassis_e_ctrl.total_current_limit / chassis_e_ctrl.total_current;
								chassis_power_control->motor_speed_pid[0].out*=current_scale;
								chassis_power_control->motor_speed_pid[1].out*=current_scale;
								chassis_power_control->motor_speed_pid[2].out*=current_scale;
								chassis_power_control->motor_speed_pid[3].out*=current_scale;
	//						}
							chassis_e_ctrl.adp_pwr_ctrl_result_status = adp_cpc_MAX_loop_cnt_reached;
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
					chassis_e_ctrl.adp_pwr_ctrl_result_status = adp_cpc_NORMAL;
					break;
				}
			}
		}
		
		//values and FSM for debug regarding speed-adaptive power ctrl algorithm
		if(chassis_e_ctrl.adp_pwr_ctrl_result_status == adp_cpc_MAX_loop_cnt_reached)
		{
			chassis_e_ctrl.num_loop_limit_reached++;
		}
		else
		{
			if(chassis_e_ctrl.current_loop_cnt != 0)
			{
				chassis_e_ctrl.num_of_normal_loop++;
			}
			
			if(chassis_e_ctrl.current_loop_cnt > chassis_e_ctrl.max_speed_adp_loop_cnt)
			{
				chassis_e_ctrl.max_speed_adp_loop_cnt = chassis_e_ctrl.current_loop_cnt;
			}
		}
		
		//values for debug
		chassis_e_ctrl.motor_final_current[0] = chassis_power_control->motor_speed_pid[0].out;
		chassis_e_ctrl.motor_final_current[1] = chassis_power_control->motor_speed_pid[1].out;
		chassis_e_ctrl.motor_final_current[2] = chassis_power_control->motor_speed_pid[2].out;
		chassis_e_ctrl.motor_final_current[3] = chassis_power_control->motor_speed_pid[3].out;
		
		chassis_e_ctrl.motor_final_total_current = 0;
		for(uint8_t i = 0; i < 4; i++)
		{
			chassis_e_ctrl.motor_final_total_current += fabs(direct_connect_e_cont.motor_final_current[i]);
		}

}

//调控速度; 速度自适应的 功率控制; 结合 分段常数控制器
void speed_adaptive_chassis_power_control(chassis_move_t *chassis_power_control)
{
	  //fp32 current_scale;
	
		direct_connect_e_cont.robot_id = get_robot_id();
	
		/*---更新一下需要用到的 动态变动的数据---*/
	  direct_connect_e_cont.chassis_power_limit = get_chassis_power_limit();
	  if(direct_connect_e_cont.chassis_power_limit>MAX_REASONABLE_CHARGE_PWR) //( (direct_connect_e_cont.chassis_power_limit>MAX_REASONABLE_CHARGE_PWR) || (direct_connect_e_cont.chassis_power_limit <0) )
		{//识别 并处理 不合理数值
			direct_connect_e_cont.chassis_power_limit = 50;
		}
		
		//从裁判系统获取当前缓冲能量
		get_chassis_power_and_buffer(&direct_connect_e_cont.chassis_power, &direct_connect_e_cont.chassis_power_buffer);
		
		//识别 并处理 chassis_power 和 chassis_power_buffer 不合理数值；--- SZL: 暂时不处理 ---
		
		//judge output cut-off point based on remaining energy and set the buffer ene critical val point
		/* ENERGY_BUFF_OUTPUT_CUTOFF_POINT = 3, 6; MINIMUM_ENERGY_BUFF=10, 13*/
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
			  这个问题在Hero上通过 分段常数控制器 已经解决了7-20之前测试都没问题
			
			7-20晚上: 第一次测试步兵的时候, 把步兵架在架子上, 未使用超级电容 轮子空转 向前全速跑 有时会出现超功率扣血, 可能是cut-off不及时; 后来安装了 裁判系统超级电容管理模块
			可是在7-21的相同测试中该问题并未复现
			
			
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

static void map_superCap_charge_pwr_to_debuff_total_current_limit(uint16_t charge_pwr, fp32* total_i_lim)
{
#ifdef HERO_CHASSIS_POWER_CONTROL
		if(charge_pwr<=30)
		{
			*total_i_lim = 8333.33f;//200w
		}
		else if(charge_pwr>30 && charge_pwr<=50)
		{
			*total_i_lim = 10000.0f;//240w
		}
		else if(charge_pwr>50 && charge_pwr<=70)
		{
			*total_i_lim = 16000.0f;//384w
		}
		else if(charge_pwr>70 && charge_pwr<=90)
		{
			*total_i_lim = 16000.0f;
		}
		else if(charge_pwr>90 && charge_pwr<=120)
		{
			*total_i_lim = 16000.0f;
		}
		else
	  {//一个典型值 = 10000
			*total_i_lim = 10000.0f;
		}
#else
		if(charge_pwr<=30)
		{
			*total_i_lim = 13000.0f;//10000.0f; //8333.33f;//200w
		}
		else if(charge_pwr>40 && charge_pwr<=50)
		{
			*total_i_lim = 14500.0f;//13000.0f;//240w
		}
		else if(charge_pwr>50 && charge_pwr<=60)
		{
			*total_i_lim = 16000.0f;//14500.0f;//240w
		}
		else if(charge_pwr>60 && charge_pwr<=80)
		{
			*total_i_lim = 20000.0f;//384w
		}
		else if(charge_pwr>80 && charge_pwr<=100)
		{
			*total_i_lim = 22000.0f;
		}
		else
	  {//一个典型值 = 10000
			*total_i_lim = 10000.0f;
		}
#endif
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
	  if(direct_connect_e_cont.chassis_power_limit > MAX_REASONABLE_CHARGE_PWR)
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
