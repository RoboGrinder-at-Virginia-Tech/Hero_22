
/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       referee_usart_task.c/h
  * @brief      RM referee system data solve. RM裁判系统数据处理
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef REFEREE_USART_TASK_H
#define REFEREE_USART_TASK_H
#include "main.h"
#include "struct_typedef.h"

#define USART_RX_BUF_LENGHT     512
#define REFEREE_FIFO_BUF_LENGTH 1024
#define PC_FIFO_BUF_LENGTH 1024

/**
  * @brief          referee task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          裁判系统任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void referee_usart_task(void const * argument);

/*
@brief         打包发送机器人数据    （UART1)
                Rui Peng 2021/2/25
								测试版本
*/
void sendPack(uint8_t cmd_ID,uint8_t level, uint8_t robot_ID);
	
/*
@brief  各种CV——EE/通讯配置
*/
/*
Autoaim delta 坐标发回TypeC板
Auto fire LOCK 绝对坐标发回TypeC
*/

#define HEADER 0xaf //包头
#define AUTOFIRE 0xff //自动开火
#define AUTOAIM 0x00 //自动瞄准
#define MANUEL  0xfa //纯手动模式
#define BIGBUFF 0xbb //打大符
#define SINBIGBUFF 0xaa //变速大符

#define LEVEL_I  0x01 //1级
#define LEVEL_II 0x02 //2级
#define LEVEL_III 0x03 //3级

#define ROBOTID_RED 0xff  //己方机器人颜色
#define ROBOTID_BLUE 0x00  //己方机器人颜色

/*
//miniPC 状态机

//发给Cap的数据
	uint8_t max_charge_pwr_command;
	uint8_t fail_safe_charge_pwr_command;
	
//接收到的 经过转换后的数据
float EBPct_fromCap; //这个是相对于0J 0%-100%
*/
//SZL 5-31-2022 EE->CV
struct typeC_msg_to_PC_t
{
		uint8_t header;
		uint8_t cmd_id;
		uint16_t shooter_speed;//PID estimate speed
		uint8_t robot_id;
		//uint16_t (*INS_pc_quat)[4];数组指针, 指向数组的指针
		uint16_t INS_pc_quat[4];
		uint8_t checksum;
};

/*
@brief         与上位机通讯  任务   （发送） （UART1)
                Rui Peng 2021/2/25
								测试版本
*/
void sendData_Task_EE_To_PC(void);




/*
@brief         上位机通讯 任务  接收 初始化（UART1)
                Rui Peng 2021/2/25
								测试版本
*/
void pc_control_init(void);


/*
@brief         上位机通讯 任务  （UART1)
                Rui Peng 2021/2/25
								测试版本
*/


//之前是8 现在是11个字节
#define PACK_LENG 11
#define GIMBAL_MOVEMENT 1
//buffer中的index
#define CMDID_INDEX 1
#define CV_status 1
#define YAW_INDEX_MSB 2
#define YAW_INDEX_LSB 3
#define PITCH_INDEX_MSB 4
#define PITCH_INDEX_LSB 5
#define DISTANCE_INDEX 6
#define SHOOT_INDEX 7

//uint8_t  autoAimFlag = 1; //自动瞄准开关状态 0关 1自动瞄准
//uint8_t shootCommand = 0x00;//自动开火指令  0x00 = 停火  0xff = 开火
//uint8_t fricCommand = 0x01;// 摩擦轮转速指令  0x01 =低转  0x02 = 高转
//fp32 yawMove = 0;  //云台数据
//fp32 pitchMove = 0;  //云台数据
//uint32_t timeFlag = 0; //
typedef enum
{
	miniPC_offline,
	miniPC_online
}miniPC_connection_status_e;//这两个结构体 不同的超级电容共用
typedef struct
{
	//MiniPC 那边发过来的信息
	uint8_t cmd_id;
	uint8_t cv_status; //0自瞄关闭, 1AID, 2LOCK
	int16_t yawCommand; //delta yaw
	int16_t pitchCommand; //delta pitch
	uint16_t dis_raw;
	uint8_t shootCommand;//自动开火指令  0x00 = 停火  0xff = 开火
	
	
	//其它FSM 以及 数据
	fp32 dis; //单位转换之后的距离
	/*如果距离是0 就说明没有识别到目标*/
	uint8_t autoAimFlag; //自动瞄准开关状态 0关 1自动瞄准AID 2自动瞄准LOCK
	
	uint8_t enemy_detected;//0 未识别到, 1识别到了
	
	fp32 yawMove_aid;
	fp32 pitchMove_aid;
	
	fp32 yawMove_absolute;
	fp32 pitchMove_absolute;
	
	miniPC_connection_status_e miniPC_connection_status;
}miniPC_info_t;

//struct gimbal_cmd
//{
//	uint8_t head;
//	uint8_t id;
//	int16_t pitch;
//	int16_t yaw;
//};

#endif
