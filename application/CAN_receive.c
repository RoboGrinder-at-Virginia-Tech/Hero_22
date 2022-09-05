/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "CAN_receive.h"

#include "cmsis_os.h"

#include "main.h"
#include "bsp_rng.h"


#include "detect_task.h"

//SZL 3-12-2022
#include "SuperCap_comm.h"

extern superCap_info_t superCap_info;
extern wulieCap_info_t wulie_Cap_info;
extern supercap_can_msg_id_e current_superCap;


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/*
SZL-1-6-2021
CAN1 FIFO 0 中断向量的 callback 用户函数
CAN2 FIFO 1 中断向量的 callback 用户函数
*/
void userCallback_CAN1_FIFO0_IT(CAN_HandleTypeDef *hcan);

void userCallback_CAN2_FIFO1_IT(CAN_HandleTypeDef *hcan);

//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
/*原始的注释 from DJI 仅供参考
motor data,  0:chassis motor1 3508;1:chassis motor3 3508;2:chassis motor3 3508;3:chassis motor4 3508;
4:yaw gimbal motor 6020;5:pitch gimbal motor 6020;6:trigger motor 2006;
电机数据, 0:底盘电机1 3508电机,  1:底盘电机2 3508电机,2:底盘电机3 3508电机,3:底盘电机4 3508电机;
4:yaw云台电机 6020电机; 5:pitch云台电机 6020电机; 6:拨弹电机 2006电机
-------------------------------------

		
*/
static motor_measure_t motor_chassis[10];//应该为[8]?弄成10防止越界
union debug_result
{
	float d;
	uint8_t data[4];
}debug_r2;
/*
		绑定在CAN2上的电机 数组
		
		*/
static motor_measure_t motor_CAN2Bus[10];

static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];

uint8_t rx_data_debug[8] = {0,0,0,0,0,0,0,0};

/*
所有电机数据的接收, 转子端方向的判断
下面那几句话的目的是判断电机方向的, 对码盘值进行积分时, 需要确定那一帧step的方向, 不能用rpm来, rpm是瞬时的
*/
void get_motor_measure_new(motor_measure_t* ptr, uint8_t data[])
{
//    (ptr)->last_ecd = (ptr)->ecd;
//    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);
//    (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);
//    (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);
//    (ptr)->temperate = (data)[6];
  
		ptr->last_ecd = ptr->ecd;
    ptr->ecd = (uint16_t)( data[0]<<8 | data[1] );
    ptr->speed_rpm = (uint16_t)( data[2]<<8 | data[3] );
    ptr->given_current = (uint16_t)( data[4]<<8 | data[5] );
    ptr->temperate = data[6];

    int relative_ecd = ptr->ecd - ptr->last_ecd;
    if (relative_ecd > 4096)//half_ecd_range=4096
    {
        relative_ecd -= 8191;//ecd_range=8191
    }
    else if (relative_ecd < -4096)
    {
        relative_ecd += 8191;
    }

    ptr->delta_ecd = relative_ecd;
    ptr->total_ecd += ptr->delta_ecd;//添加：开机 offset

}

/**
  * @brief          hal CAN fifo call back, receive motor data
		SZL 12-30-2021添加 FIFO0 仅 CAN1使用
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    userCallback_CAN1_FIFO0_IT(hcan);
}

/**
  * @brief          hal CAN fifo1 call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal库CAN fifo回调函数,接收电机数据 在bsp_can.c中 将fifo1 与 CAN2绑定到一起了
	SZL 12-30-2021添加 FIFO1 仅 CAN2使用
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    userCallback_CAN2_FIFO1_IT(hcan);
}


/**
  * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
  * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000] 
  * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
  * @param[in]      shoot: (0x207) 2006 motor control current, range [-10000,10000]
  * @param[in]      rev: (0x208) reserve motor control current
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      rev: (0x208) 保留，电机控制电流
  * @retval         none
  */
void CAN_cmd_gimbal(int16_t pitch, int16_t rev)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = 0;//(yaw >> 8);
    gimbal_can_send_data[1] = 0;//yaw;
    gimbal_can_send_data[2] = (pitch >> 8);
    gimbal_can_send_data[3] = pitch;
   // gimbal_can_send_data[4] = (shoot >> 8);
    //gimbal_can_send_data[5] = shoot;
	gimbal_can_send_data[4] = 0;
	gimbal_can_send_data[5] = 0;
    gimbal_can_send_data[6] = (rev >> 8);
    gimbal_can_send_data[7] = rev;
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}
/**
  * @brief          发送电机控制电流(0x207)
  * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @retval         none
  */
void CAN_cmd_gimbal2(int16_t shoot,int16_t yaw, int16_t shoot_17mm){
	
		uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;
    gimbal_can_send_data[2] = 0;
    gimbal_can_send_data[3] = 0;
    gimbal_can_send_data[4] = (shoot >> 8);
    gimbal_can_send_data[5] = shoot;
		gimbal_can_send_data[6] = (shoot_17mm >> 8);
		gimbal_can_send_data[7] = shoot_17mm;
		HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

//新Hero CAN2所有设备
void CAN_cmd_gimbal_new_Hero_gimbal(int16_t pitch, int16_t yaw, int16_t shoot_42mm, int16_t shoot_17mm)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;
    gimbal_can_send_data[2] = (pitch >> 8);
    gimbal_can_send_data[3] = pitch;
		gimbal_can_send_data[4] = (shoot_42mm >> 8);
    gimbal_can_send_data[5] = shoot_42mm;
		gimbal_can_send_data[6] = (shoot_17mm >> 8);
		gimbal_can_send_data[7] = shoot_17mm;
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}
/**
  * @brief          send control current of motor (0x208, 0x209)
  * @param[in]      motor1: (0x208) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x209) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x208) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x209) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
/*
12-30-2021 SZL
发送电机控制电流 0x201LEFT 0x202RIGHT 
*/
void CAN_cmd_friction_wheel(int16_t motor1, int16_t motor2){
	
		uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (motor1 >> 8);//(m1>>8)
    gimbal_can_send_data[1] = motor1;//m1
    gimbal_can_send_data[2] = (motor2 >> 8);
    gimbal_can_send_data[3] = motor2;
    gimbal_can_send_data[4] = 0; //(m3>>8);//(m3>>8)
    gimbal_can_send_data[5] = 0; //m3;//m3
    gimbal_can_send_data[6] = 0; //(m4>>8);//(m4>>8)
    gimbal_can_send_data[7] = 0; //m4;//m4
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}
/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回yaw 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &motor_CAN2Bus[4];
}

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回pitch 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    //return &motor_chassis[5];//SZL修改 1-6-2021 由于电机在CAN 2 上 目前解包到motor_CAN2Bus数组中
		return &motor_CAN2Bus[5];
}


/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
//M3508 42mm 拨盘电机
const motor_measure_t *get_trigger_motor_measure_point(void)
{
    //return &motor_chassis[6];
		//SZL修改 6-15-2022 由于电机在CAN 2 上 目前解包到motor_CAN2Bus数组中
		return &motor_CAN2Bus[6];
}

const motor_measure_t *get_trigger_motor_measure_17mm_point(void)
{
		return &motor_CAN2Bus[7];
		//return &motor_chassis[7];
}


/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}

/*
SZL 12-30-2021 添加 M3508 摩擦轮 shooter 两个 left_friction_motor_measure right_friction_motor_measure
*/
const motor_measure_t *get_left_friction_motor_measure_point(void)
{
	return &motor_CAN2Bus[0];
}
const motor_measure_t *get_right_friction_motor_measure_point(void)
{
	return &motor_CAN2Bus[1];
}

/*
SZL-1-6-2021
CAN1 FIFO 0 中断向量的 callback 用户函数
记得注释掉void CAN1_RX0_IRQHandler(void)中的HAL_CAN_IRQHandler(..)
*/
void userCallback_CAN1_FIFO0_IT(CAN_HandleTypeDef *hcan)
{
		CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
		//uint8_t temp = 0;
	
		if(hcan == &hcan2)
		{
			//temp = 1;
			return;
		}

		if(hcan == &hcan1)
		{
			HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
			
			static uint8_t i_can1 = 0;
			i_can1 = rx_header.StdId - CAN_3508_M1_ID;//get motor id and index in array
			switch (rx_header.StdId)
			{
					case CAN_3508_M1_ID:
					{
							//get motor id and index in array
							//i_can1 = rx_header.StdId - CAN_3508_M1_ID;
							get_motor_measure_new(&motor_chassis[i_can1], rx_data);
							detect_hook(CHASSIS_MOTOR1_TOE);
							break;
					}
					case CAN_3508_M2_ID:
					{
							//get motor id and index in array
							//i_can1 = rx_header.StdId - CAN_3508_M1_ID;
							get_motor_measure_new(&motor_chassis[i_can1], rx_data);
							detect_hook(CHASSIS_MOTOR2_TOE);
							break;
					}
					case CAN_3508_M3_ID:
					{
							//get motor id and index in array
							//i_can1 = rx_header.StdId - CAN_3508_M1_ID;
							get_motor_measure_new(&motor_chassis[i_can1], rx_data);
							detect_hook(CHASSIS_MOTOR3_TOE);
							break;
					}
					case CAN_3508_M4_ID:
					{
							//get motor id and index in array
							//i_can1 = rx_header.StdId - CAN_3508_M1_ID;
							get_motor_measure_new(&motor_chassis[i_can1], rx_data);
							detect_hook(CHASSIS_MOTOR4_TOE);
							break;
					}
					case SuperCap_ID:
					{
							superCap_info.msg_u_EBPct.array[1] = rx_data[0];
							superCap_info.msg_u_EBPct.array[0] = rx_data[1];
							
							superCap_info.msg_u_VBKelvin.array[1] = rx_data[2];
							superCap_info.msg_u_VBKelvin.array[0] = rx_data[3];
						
							superCap_info.EBPct_fromCap = (float)(superCap_info.msg_u_EBPct.msg_u / 100.0f);
							superCap_info.VBKelvin_fromCap = (float)(superCap_info.msg_u_VBKelvin.msg_u / 100.0f);
						
							
							
							debug_r2.data[0] = rx_data_debug[3];
							debug_r2.data[1] = rx_data_debug[2];
							debug_r2.data[2] = rx_data_debug[1];
							debug_r2.data[3] = rx_data_debug[0];
							
							detect_hook(SUPERCAP_TOE);
							break;
					}
					case wulie_Cap_CAN_ID:
					{
							current_superCap = wulie_Cap_CAN_ID;
						  uint16_t* pPowerdata = (uint16_t *) rx_data;//------------------------------------
							wulie_Cap_info.PowerData[0] = (fp32)pPowerdata[0] / 100.0f;
							wulie_Cap_info.PowerData[1] = (fp32)pPowerdata[1] / 100.0f;
							wulie_Cap_info.PowerData[2] = (fp32)pPowerdata[2] / 100.0f;
							wulie_Cap_info.PowerData[3] = (fp32)pPowerdata[3] / 100.0f;
						
							wulie_Cap_info.input_voltage = wulie_Cap_info.PowerData[0]; //输入电压
							wulie_Cap_info.cap_voltage = wulie_Cap_info.PowerData[1];//电容电压
							wulie_Cap_info.input_current = wulie_Cap_info.PowerData[2];//输入电流
							wulie_Cap_info.set_power = wulie_Cap_info.PowerData[3];//设定功率
						
							//计算容量
							wulie_Cap_info.EBank = 0.5f * wulie_Cap_info.cap_voltage * wulie_Cap_info.cap_voltage * CAPACITY_WULIE_CAP;//CAPACITY=6
						  wulie_Cap_info.EBPct = (wulie_Cap_info.cap_voltage * wulie_Cap_info.cap_voltage)/(CHARACTERISTIC_VOLTAGE_WULIE_CAP * CHARACTERISTIC_VOLTAGE_WULIE_CAP)*100.0f;
							detect_hook(WULIE_CAP_TOE);
							break;
					}

					default:
					{
							break;
					}
			}
		}
	
	
}

/*
SZL-1-6-2021
CAN2 FIFO 1 中断向量的 callback 用户函数
记得注释掉void CAN2_RX1_IRQHandler(void)中的HAL_CAN_IRQHandler(..)
*/
void userCallback_CAN2_FIFO1_IT(CAN_HandleTypeDef *hcan)
{
		CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
		//uint8_t temp = 0;
	
		if(hcan == &hcan1)
		{
			//temp = 1;
			return;
		}
		
		if(hcan == &hcan2)
		{
			HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data);
			
			static uint8_t i_can2 = 0;
			i_can2 = rx_header.StdId - CAN_SHOOTL_ID;
			switch (rx_header.StdId)
			{
					case CAN_SHOOTL_ID:
					{
							get_motor_measure_new(&motor_CAN2Bus[i_can2], rx_data);
						  detect_hook(SHOOT_FRIC_L_TOE);
							break;
					}
					case CAN_SHOOTR_ID:
					{
							get_motor_measure_new(&motor_CAN2Bus[i_can2], rx_data);
						  detect_hook(SHOOT_FRIC_R_TOE);
							break;
					}
					case CAN_YAW_MOTOR_ID:
					{
							get_motor_measure_new(&motor_CAN2Bus[i_can2], rx_data);
						  detect_hook(YAW_GIMBAL_MOTOR_TOE);
							break;
					}
					case CAN_PIT_MOTOR_ID:
					{
							get_motor_measure_new(&motor_CAN2Bus[i_can2], rx_data);
						  detect_hook(PITCH_GIMBAL_MOTOR_TOE);
							break;
					}
					case CAN_TRIGGER_MOTOR_ID:
					{
							get_motor_measure_new(&motor_CAN2Bus[i_can2], rx_data);
						  detect_hook(TRIGGER_MOTOR_TOE);
							break;
					}
					case CAN_TRIGGER_MOTOR_17mm_ID:
					{
							get_motor_measure_new(&motor_CAN2Bus[i_can2], rx_data);
						  detect_hook(TRIGGER_MOTOR_17mm_TOE);
							break;
					}
					default:
					{
							break;
					}
			}
		}	
	
}
