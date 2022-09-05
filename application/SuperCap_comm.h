#ifndef __SUPERCAP_COMM_H___
#define __SUPERCAP_COMM_H___

#include "struct_typedef.h"
#include "CAN_receive.h"

#define SUPERCAP_CAN hcan1
///*以下两个Texas不使用*/
//#define ICRA_superCap_max_power 65//40
//#define ICRA_superCap_fail_safe_power 65//40

/*超级电容电容组数据*/
#define CHARACTERISTIC_VOLTAGE_WULIE_CAP 24.0f//26.5f //特征电压是充电电路输出电压，对于C620必须低一些，否则会触发保护
#define CAPACITY_WULIE_CAP 6.0f //电容容量，单位法拉

#define CHARACTERISTIC_VOLTAGE_ZIDA_CAP 24.0f
#define CAPACITY_ZIDA_CAP 6.0f

extern uint8_t debug_max_pwr;
extern uint8_t debug_fail_safe_pwr;
extern void CAN_command_superCap(uint8_t max_pwr, uint8_t fail_safe_pwr);
extern void CAN_command_wulie_Cap(uint16_t temPower);
extern void superCap_offline_proc(void);
extern bool_t superCap_is_data_error_proc(void);
extern void superCap_solve_data_error_proc(void);
extern void superCap_comm_bothway_init(void);
extern void superCap_control_loop(void);

extern void wulie_Cap_offline_proc(void);
extern bool_t wulie_Cap_is_data_error_proc(void);
extern void get_superCap_vol_and_energy(fp32* cap_voltage, fp32* EBank);
extern uint16_t get_superCap_charge_pwr(void);
extern bool_t current_superCap_is_offline(void);
extern bool_t all_superCap_is_offline(void);

typedef enum
{
	//TypeC -> SuperCap 时 CAN报文 即指令的ID
	 RMTypeC_Master_Command_ID = 0x4FF,
	//SuperCap -> TypeC时 CAN报文 即反馈报文的ID:
	 SuperCap_ID = 0x500,
	
	//雾列的控制板
	//TypeC -> wulie Cap
	RMTypeC_Master_Command_ID_for_WuLie = 0x210,
	
	//wulie Cap -> TypeC
	wulie_Cap_CAN_ID = 0x211,
}supercap_can_msg_id_e;

typedef enum
{
	superCap_offline,
	superCap_online
}superCap_connection_status_e;//这两个结构体 不同的超级电容共用

typedef enum
{
	SuperCap_dataIsError,
	SuperCap_dataIsCorrect
}superCap_dataError_status_e;//这两个结构体 不同的超级电容共用

typedef struct
{
	//发给Cap的数据
	uint8_t max_charge_pwr_command;
	uint8_t fail_safe_charge_pwr_command;
	
	//接收到的 经过转换后的数据
	float EBPct_fromCap; //这个是相对于0J 0%-100%
	float VBKelvin_fromCap;//
	superCap_connection_status_e status;
	
//	int32_t EBank;
	fp32 EBank;
	
	superCap_dataError_status_e data_EBPct_status;
	union superCap_msg_u
	{
		uint16_t msg_u;
		/* 小端模式下: msg_u_EBPct高字节位对应的是a[1]; 低字节位对应的是a[0] */
		uint8_t array[2];
	}msg_u_EBPct, msg_u_VBKelvin;
	
	uint8_t a;
	uint8_t b;
	uint8_t c;
	
}superCap_info_t;

typedef struct
{
	//发给cap的数据
	uint16_t max_charge_pwr_from_ref;
	uint16_t charge_pwr_command;
	
	//接受到的 经过转换后的数据
	fp32 PowerData[4];
	fp32 input_voltage;//输入电压
  fp32 cap_voltage;//电容电压
	fp32 input_current; //输入电流
	fp32 set_power;//设定功率
	
	//其它FSM 和 变量
	superCap_connection_status_e status;
	//这个是相对于0J 0%-100%
	fp32 EBPct;
	fp32 EBank;

}wulieCap_info_t;

extern superCap_info_t superCap_info;
extern uint8_t debug_a;
extern uint8_t debug_b;
extern uint8_t debug_c;

#endif /*__SUPERCAP_COMM_H___*/
