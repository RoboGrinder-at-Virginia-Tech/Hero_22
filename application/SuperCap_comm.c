//3-10-2022增加Cap和C板的通信 Can 1 经过滑环

#include "SuperCap_comm.h"
#include "main.h"
#include "cmsis_os.h"
#include "CAN_receive.h"
#include "referee.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void superCap_offline_proc(void);

//static CAN_TxHeaderTypeDef  superCap_tx_message;
static uint8_t              superCap_can_send_data[8];
static uint8_t              wulieCap_can_send_data[8];
superCap_info_t superCap_info1;
superCap_info_t superCap_info2;
superCap_info_t superCap_info3;//<----越界指针边界 大致位置
superCap_info_t superCap_info4;
superCap_info_t superCap_info;//用的这一个 3 4和这个是正常的
wulieCap_info_t wulie_Cap_info;//雾列超级电容控制板的结构体
CAN_TxHeaderTypeDef  superCap_tx_message;
CAN_TxHeaderTypeDef  wulie_Cap_tx_message;
supercap_can_msg_id_e current_superCap; //表明当前使用的是哪一个超级电容

uint8_t debug_max_pwr;
uint8_t debug_fail_safe_pwr;
//uint8_t debug_a=0;
//uint8_t debug_b;
//uint8_t debug_c;

uint32_t any_Cap_can_msg_send_TimeStamp = 0;
const uint16_t any_Cap_can_msg_send_sendFreq = 100;

void superCap_comm_bothway_init()
{
	/*
	初始化以下:
	1 CAN发送数据
	2 CAN接收数据
	*/
	//1初始化发送
	superCap_info.max_charge_pwr_command = 0;
	superCap_info.fail_safe_charge_pwr_command = 0;
	
	//2初始化接收
	superCap_info.EBPct_fromCap = 0.0f;
	superCap_info.VBKelvin_fromCap = 0.0f;
	superCap_info.status = superCap_offline;
	//superCap_info.data_EBPct_status = SuperCap_dataIsError;
	superCap_info.msg_u_EBPct.array[0] = 0;
	superCap_info.msg_u_EBPct.array[1] = 0;
	superCap_info.a = 0;
	superCap_info.b = 0;
	superCap_info.c = 0;
	
	current_superCap = wulie_Cap_CAN_ID;
}


void superCap_control_loop()
{
	//发送任务计时, 时间到了开始一次发送
	if(xTaskGetTickCount() - any_Cap_can_msg_send_sendFreq > any_Cap_can_msg_send_TimeStamp)
	{
		any_Cap_can_msg_send_TimeStamp = xTaskGetTickCount(); //更新时间戳 
			
		if(current_superCap == SuperCap_ID)
		{//紫达控制板
			//Texas 调试
			superCap_info.max_charge_pwr_command = get_chassis_power_limit() - 2.5f;
				
			//这时fail safe该不该继续按照当前允许的最大功率来SZL 5-16-2022 还是应该按等级信息来算?
			superCap_info.fail_safe_charge_pwr_command = get_chassis_power_limit() - 2.5f;
				
			if(superCap_info.max_charge_pwr_command >= 101.0f)
			{
				superCap_info.max_charge_pwr_command = 40;
			}
				
			if(superCap_info.fail_safe_charge_pwr_command >= 101.0f)
			{
				superCap_info.fail_safe_charge_pwr_command = 40;
			}
				
			CAN_command_superCap(superCap_info.max_charge_pwr_command, superCap_info.fail_safe_charge_pwr_command);	
		}
		else
		{//雾列控制板
			wulie_Cap_info.max_charge_pwr_from_ref = get_chassis_power_limit() - 2.5f;
				
			if(wulie_Cap_info.max_charge_pwr_from_ref > 101.0f)
			{//超出合理范围时, 充电功率设置为40
				wulie_Cap_info.max_charge_pwr_from_ref = 40;
			}
				
			wulie_Cap_info.charge_pwr_command = wulie_Cap_info.max_charge_pwr_from_ref * 100.f;
			CAN_command_wulie_Cap(wulie_Cap_info.charge_pwr_command);
		}
	}
}
//	//ICRA only
//	superCap_info.max_charge_pwr_command = ICRA_superCap_max_power; //=65w
//	superCap_info.fail_safe_charge_pwr_command = ICRA_superCap_fail_safe_power; //=65w
//	CAN_command_superCap(superCap_info.max_charge_pwr_command, superCap_info.fail_safe_charge_pwr_command);	

/*
SZL 3-10-2022 下发到SuperCap的数据
*/
void CAN_command_superCap(uint8_t max_pwr, uint8_t fail_safe_pwr)
{
		uint32_t send_mail_box;
    superCap_tx_message.StdId = RMTypeC_Master_Command_ID;
    superCap_tx_message.IDE = CAN_ID_STD;
    superCap_tx_message.RTR = CAN_RTR_DATA;
    superCap_tx_message.DLC = 0x08;
    superCap_can_send_data[0] = max_pwr;
    superCap_can_send_data[1] = fail_safe_pwr;
    superCap_can_send_data[2] = 0;
    superCap_can_send_data[3] = 0;
    superCap_can_send_data[4] = 0; 
    superCap_can_send_data[5] = 0; 
    superCap_can_send_data[6] = 0; 
    superCap_can_send_data[7] = 0; 
    HAL_CAN_AddTxMessage(&SUPERCAP_CAN, &superCap_tx_message, superCap_can_send_data, &send_mail_box);
}

void CAN_command_wulie_Cap(uint16_t temPower)
{
		uint32_t send_mail_box;
    wulie_Cap_tx_message.StdId = RMTypeC_Master_Command_ID_for_WuLie;
    wulie_Cap_tx_message.IDE = CAN_ID_STD;
    wulie_Cap_tx_message.RTR = CAN_RTR_DATA;
    wulie_Cap_tx_message.DLC = 0x08;
    wulieCap_can_send_data[0] = temPower >> 8;
    wulieCap_can_send_data[1] = temPower;
    wulieCap_can_send_data[2] = 0;
    wulieCap_can_send_data[3] = 0;
    wulieCap_can_send_data[4] = 0; 
    wulieCap_can_send_data[5] = 0; 
    wulieCap_can_send_data[6] = 0; 
    wulieCap_can_send_data[7] = 0; 
    HAL_CAN_AddTxMessage(&hcan1, &wulie_Cap_tx_message, wulieCap_can_send_data, &send_mail_box);
}

//返回数据相关
void superCap_offline_proc()
{
		superCap_info.status = superCap_offline;
		
	
}

bool_t superCap_is_data_error_proc()
{
		superCap_info.status = superCap_online;
	
		//ICRA
		//只判断EBPct_fromCap，因为只用这个, 这个是0% 这种 并不是0.0 - 100.0
		//注意EBPct是可能超过100%的所以暂时把检查数据是否出错改成这个样子 -100.0 ~ 200.0
		//下面的superCap_solve_data_error_proc也有更改
		if(superCap_info.EBPct_fromCap < -100.0f || superCap_info.EBPct_fromCap > 200.0f)
		{
			superCap_info.data_EBPct_status = SuperCap_dataIsError;
			return 0;
			
		}
		else
		{
			superCap_info.data_EBPct_status = SuperCap_dataIsCorrect;
			return 0;
		}
}

void superCap_solve_data_error_proc()
{
		//因为其数值可能超过100 所以暂时把这个功能屏蔽掉 ICRA
//		if(superCap_info.data_EBPct_status == SuperCap_dataIsError)
//		{
//			if(superCap_info.EBPct_fromCap < 0.0f)
//				superCap_info.EBPct_fromCap = 0.0f;
//			if(superCap_info.EBPct_fromCap > 100.0f)
//				superCap_info.EBPct_fromCap = 100.0f;
//		}
		return;
}

//以下为雾列相关的
void wulie_Cap_offline_proc()
{
		wulie_Cap_info.status = superCap_offline;
}

bool_t wulie_Cap_is_data_error_proc()
{
		wulie_Cap_info.status = superCap_online;
		//永远 return 0;
		return 0;
//		//ICRA
//		//只判断EBPct_fromCap，因为只用这个, 这个是0% 这种 并不是0.0 - 100.0
//		//注意EBPct是可能超过100%的所以暂时把检查数据是否出错改成这个样子 -100.0 ~ 200.0
//		//下面的superCap_solve_data_error_proc也有更改
//		if(superCap_info.EBPct_fromCap < -100.0f || superCap_info.EBPct_fromCap > 200.0f)
//		{
//			superCap_info.data_EBPct_status = SuperCap_dataIsError;
//			return 1;
//			
//		}
//		else
//		{
//			superCap_info.data_EBPct_status = SuperCap_dataIsCorrect;
//			return 0;
//		}
}
