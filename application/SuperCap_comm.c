//3-10-2022����Cap��C���ͨ�� Can 1 ��������

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
superCap_info_t superCap_info3;//<----Խ��ָ��߽� ����λ��
superCap_info_t superCap_info4;
superCap_info_t superCap_info;//�õ���һ�� 3 4�������������
wulieCap_info_t wulie_Cap_info;//���г������ݿ��ư�Ľṹ��
CAN_TxHeaderTypeDef  superCap_tx_message;
CAN_TxHeaderTypeDef  wulie_Cap_tx_message;
supercap_can_msg_id_e current_superCap; //������ǰʹ�õ�����һ����������

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
	��ʼ������:
	1 CAN��������
	2 CAN��������
	*/
	//1��ʼ������
	superCap_info.max_charge_pwr_command = 0;
	superCap_info.fail_safe_charge_pwr_command = 0;
	
	//2��ʼ������
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
	//���������ʱ, ʱ�䵽�˿�ʼһ�η���
	if(xTaskGetTickCount() - any_Cap_can_msg_send_sendFreq > any_Cap_can_msg_send_TimeStamp)
	{
		any_Cap_can_msg_send_TimeStamp = xTaskGetTickCount(); //����ʱ��� 
			
		if(current_superCap == SuperCap_ID)
		{//�ϴ���ư�
			//Texas ����
			superCap_info.max_charge_pwr_command = get_chassis_power_limit() - 2.5f;
				
			//��ʱfail safe�ò��ü������յ�ǰ������������SZL 5-16-2022 ����Ӧ�ð��ȼ���Ϣ����?
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
		{//���п��ư�
			wulie_Cap_info.max_charge_pwr_from_ref = get_chassis_power_limit() - 2.5f;
				
			if(wulie_Cap_info.max_charge_pwr_from_ref > 101.0f)
			{//��������Χʱ, ��繦������Ϊ40
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
SZL 3-10-2022 �·���SuperCap������
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

//�����������
void superCap_offline_proc()
{
		superCap_info.status = superCap_offline;
		
	
}

bool_t superCap_is_data_error_proc()
{
		superCap_info.status = superCap_online;
	
		//ICRA
		//ֻ�ж�EBPct_fromCap����Ϊֻ�����, �����0% ���� ������0.0 - 100.0
		//ע��EBPct�ǿ��ܳ���100%��������ʱ�Ѽ�������Ƿ����ĳ�������� -100.0 ~ 200.0
		//�����superCap_solve_data_error_procҲ�и���
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
		//��Ϊ����ֵ���ܳ���100 ������ʱ������������ε� ICRA
//		if(superCap_info.data_EBPct_status == SuperCap_dataIsError)
//		{
//			if(superCap_info.EBPct_fromCap < 0.0f)
//				superCap_info.EBPct_fromCap = 0.0f;
//			if(superCap_info.EBPct_fromCap > 100.0f)
//				superCap_info.EBPct_fromCap = 100.0f;
//		}
		return;
}

//����Ϊ������ص�
void wulie_Cap_offline_proc()
{
		wulie_Cap_info.status = superCap_offline;
}

bool_t wulie_Cap_is_data_error_proc()
{
		wulie_Cap_info.status = superCap_online;
		//��Զ return 0;
		return 0;
//		//ICRA
//		//ֻ�ж�EBPct_fromCap����Ϊֻ�����, �����0% ���� ������0.0 - 100.0
//		//ע��EBPct�ǿ��ܳ���100%��������ʱ�Ѽ�������Ƿ����ĳ�������� -100.0 ~ 200.0
//		//�����superCap_solve_data_error_procҲ�и���
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
