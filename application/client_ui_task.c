/*************************************************************

RM自定义UI协议       基于RM2020学生串口通信协议V1.3

弗吉尼亚理工 Virginia Tech; RoboGrinder

**************************************************************/


#include "client_ui_task.h"
#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>
#include "SuperCap_comm.h"
#include "shoot.h"
#include "referee.h"
#include "referee_usart_task.h"
#include "detect_task.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t client_ui_task_high_water;
#endif

extern uint8_t turboMode;
extern uint8_t swing_flag;

extern shoot_control_t shoot_control; 
extern miniPC_info_t miniPC_info;
extern supercap_can_msg_id_e current_superCap;
extern wulieCap_info_t wulie_Cap_info;

unsigned char UI_Seq;     //包序号
//uint32_t temp_time_check_RTOS = 0;
//uint32_t temp_time_check_HAL = 0;
//int _temp_a = 0;
//定义图像数据
//Graph_Data G1,G2,G3,G4,G5;
//定义图像数据 新增加 SZL 静态瞄准线
Graph_Data gAimVertL, gAimHorizL1m, gAimHorizL2m, gAimHorizL3m, gAimHorizL4m, gAimHorizL5m, left2to3, left3to4,left4to5, right2to3, right3to4,right4to5;
Graph_Data g17aim2m, g17aim3m, g17aim5m, g17aim2to3m, g17aim3to5m;

String_Data strCapVolt, strCapPct, strChassisSts, strSPINSts;//右上角
String_Data strCVSts, strGunSts, strABoxSts, strProjSLimSts, strDisSts;//左上角

//动态的东西 电压
Float_Data fCapVolt, fCapPct;
Float_Data fProjSLim, fDis; //左上角

//方框
Graph_Data gChassisSts_box, gSPINSts_box;//右上角
Graph_Data gCVSts_box, gGunSts_box, gABoxSts_box;//左上角
Graph_Data gEnemyDetected_circle, gCVfb_sts_box;

//删除图层结构体
UI_Data_Delete delLayer;

//UI信息封装结构体
ui_info_t ui_info;

uint32_t client_ui_count_ref = 0;
uint8_t client_ui_test_flag = 1;

uint32_t ui_dynamic_crt_send_TimeStamp;
const uint16_t ui_dynamic_crt_sendFreq = 1000;

void client_ui_task(void const *pvParameters)
{
		//等待referee_usart_task中完成对MCU UART6的初始化
		vTaskDelay(200);
	
//		memset(&G1,0,sizeof(G1));
//		memset(&G2,0,sizeof(G2));
//		memset(&G3,0,sizeof(G3));
//		memset(&G4,0,sizeof(G4));
//		memset(&G5,0,sizeof(G5));

		//初速绘制"创建"一次动态UI
		//动态图层 占用图层 4,5,6,7
				
		//右上角 超级电容电压
		//离线时superCap_info.VBKelvin_fromCap修改为 0
		//Float_Draw(&fCapVolt, "999", UI_Graph_ADD, 4, UI_Color_Yellow, 20, 2, 3, 1620, 810, 00.00);//superCap_info.VBKelvin_fromCap);
			Float_Draw(&fCapVolt, "999", UI_Graph_ADD, 4, UI_Color_Yellow, 20, 2, 3, 960, 250-50, 00.00);//superCap_info.VBKelvin_fromCap);//new position
		//右上角 超级电容百分比
		//离线时superCap_info.EBPct_fromCap修改为 0
	//	Float_Draw(&fCapPct, "998", UI_Graph_ADD, 4, UI_Color_Yellow, 20, 2, 3, 1620, 840, 00.00);//superCap_info.EBPct_fromCap);
			Float_Draw(&fCapPct, "998", UI_Graph_ADD, 4, UI_Color_Yellow, 40, 2, 3, 960, 200-50, 00.00);//superCap_info.EBPct_fromCap);

		//右上角方框 new position
		Rectangle_Draw(&gChassisSts_box, "997", UI_Graph_ADD, 4, UI_Color_Cyan, 3, TopRight_REC_on_NORM_START_X-80, TopRight_REC_on_NORM_START_Y-95, TopRight_REC_on_NORM_END_X-80, TopRight_REC_on_NORM_END_Y-95);
		Rectangle_Draw(&gSPINSts_box, "996", UI_Graph_ADD, 4, UI_Color_Cyan, 3, TopRight_REC_on_FOLL_START_X-80, TopRight_REC_on_FOLL_START_Y-95, TopRight_REC_on_FOLL_END_X-80, TopRight_REC_on_FOLL_END_Y-95);
		
		//左上角---还未改
		Rectangle_Draw(&gCVSts_box, "995", UI_Graph_ADD, 4, UI_Color_Cyan, 3, 240, 850, 330, 815);
		Rectangle_Draw(&gGunSts_box, "994", UI_Graph_ADD, 4, UI_Color_Cyan, 3, 240, 810, 330, 775);
		Rectangle_Draw(&gABoxSts_box, "993", UI_Graph_ADD, 4, UI_Color_Cyan, 3, 240, 770, 330, 735);
		
		//左上角 CV反馈状态
		Rectangle_Draw(&gCVfb_sts_box, "989", UI_Graph_ADD, 4, UI_Color_White, 3, TopLeft_CV_FEEDBACK_STATUS_on_OFF_START_X, TopLeft_CV_FEEDBACK_STATUS_on_OFF_START_Y, TopLeft_CV_FEEDBACK_STATUS_on_OFF_END_X, TopLeft_CV_FEEDBACK_STATUS_on_OFF_END_Y);
		
		//CV自瞄 是否瞄准到目标 圈
		Circle_Draw(&gEnemyDetected_circle, "990", UI_Graph_ADD, 4, UI_Color_Cyan, 15, TopLeft_Cir_on_cv_DET_START_X, TopLeft_Cir_on_cv_DET_START_Y, TopLeft_Cir_on_cv_DET_radius);
		
    Float_Draw(&fProjSLim, "992", UI_Graph_ADD, 4, UI_Color_Main, 20, 2, 3, 240, 720, 15.00);
		Float_Draw(&fDis, "991", UI_Graph_ADD, 4, UI_Color_Main, 20, 2, 3, 240, 680, 1.00);
		
  	UI_ReFresh(2, fCapVolt, fCapPct);
		UI_ReFresh(2, fProjSLim, fDis);
		UI_ReFresh(2, gEnemyDetected_circle, gCVfb_sts_box);
		//UI_ReFresh(5, fCapVolt, fCapPct, fProjSLim, fDis, gEnemyDetected_circle);
		UI_ReFresh(5, gChassisSts_box, gSPINSts_box, gCVSts_box, gGunSts_box, gABoxSts_box);
	  //UI 初始创建 + 发送结束
		
		//记得注释
//		miniPC_info.enemy_detected = 1;
//		miniPC_info.cv_status = 1;
//		miniPC_info.dis_raw = 0x00;
	
		/*	大装甲板宽 230mm 
		*/
		while (1)
    {
				//
//			 temp_time_check_RTOS = xTaskGetTickCount();
//			 temp_time_check_HAL = HAL_GetTick();
//			 //int _temp_a = 0;
//			 _temp_a++;
			
				//先画静态图像; 占用图层 0, 1, 2, 3
				//右上角
				//Cap Pct:字样 封装字号20 和 图形线宽 3
				//Char_Draw(&strCapPct, "008", UI_Graph_ADD, 2, UI_Color_Yellow, 20, 17, 3, CAP_PCT_X, CAP_PCT_Y,   "Cap Pct:        %");
			  //Cap Volt:字样 封装字号20 和 图形线宽 3
				//Char_Draw(&strCapVolt, "009", UI_Graph_ADD, 2, UI_Color_Yellow, 20, 9, 3, CAP_VOLT_X, CAP_VOLT_Y, "Cap Volt:");  

				//底盘状态		
				Char_Draw(&strChassisSts, "010", UI_Graph_ADD, 2, UI_Color_Yellow, 20, 10, 3, CHASSIS_STS_X-80, CHASSIS_STS_Y-95, "NORM BOOST");
				Char_Draw(&strSPINSts, "011", UI_Graph_ADD, 2, UI_Color_Yellow, 20, 9, 3, SPIN_STS_X-80, SPIN_STS_Y-95,           "FOLL SPIN");
				
			
				//中间
				//Aim 竖
				Line_Draw(&gAimVertL, "001", UI_Graph_ADD, 2, UI_Color_Orange, 1, 960+20, 540, 960+20, 0);
				
				//2m
			//	Line_Draw(&gAimHorizL2m, "003", UI_Graph_ADD, 2, UI_Color_Yellow, 1, (960-50), (540-24), (960+50), (540-24));//50
				//Line_Draw(&gAimHorizL2m, "003", UI_Graph_ADD, 2, UI_Color_Yellow, 1, (960-31), (540-66), (960+71), (540-66));// length 103x-40 size of big armor plate
				Line_Draw(&gAimHorizL2m, "003", UI_Graph_ADD, 2, UI_Color_Yellow, 1, (960-10), (540-66), (960+50), (540-66));// length 103x-40 size of small armor plate
				//3m
				//Line_Draw(&gAimHorizL3m, "004", UI_Graph_ADD, 2, UI_Color_Yellow, 1, (960-15), (540-130), (960+55), (540-130));//length 70 20+ size of big armor plate
					Line_Draw(&gAimHorizL3m, "004", UI_Graph_ADD, 2, UI_Color_Yellow, 1, (960+6), (540-130), (960+34), (540-130));//length 70 20+ size of small armor plate
				//4m (540-150) "005"
				//Line_Draw(&gAimHorizL4m, "005", UI_Graph_ADD, 2, UI_Color_Yellow, 1, (960-2), (540-227), (960+42), (540-227));//length 44 17+ size of big armor plate
				Line_Draw(&gAimHorizL4m, "005", UI_Graph_ADD, 2, UI_Color_Yellow, 1, (960+11), (540-227), (960+29), (540-227));//length 44 17+ size of small armor plate
				//5m (540-170) "006"
				//Line_Draw(&gAimHorizL5m, "006", UI_Graph_ADD, 2, UI_Color_Yellow, 1, (960+4), (540-320), (960+37), (540-320));//length 33  17+size of big armor plate
				Line_Draw(&gAimHorizL5m, "006", UI_Graph_ADD, 2, UI_Color_Yellow, 1, (960+14), (540-320), (960+27), (540-320));//length 33  17+ size of small armor plate
				// left side line drawing
				Line_Draw(&left2to3, "017", UI_Graph_ADD, 2, UI_Color_Yellow, 1, (960-31), (540-66), (960-15), (540-130) );
				Line_Draw(&left3to4, "018", UI_Graph_ADD, 2, UI_Color_Yellow, 1,(960-15), (540-130), (960-2), (540-227));
				Line_Draw(&left4to5, "019", UI_Graph_ADD, 2, UI_Color_Yellow, 1, (960-2), (540-227), (960+4), (540-320));
				
				// right side line drawing
				Line_Draw(&right2to3, "021", UI_Graph_ADD, 2, UI_Color_Yellow, 1, (960+71), (540-66), (960+55), (540-130) );
				Line_Draw(&right3to4, "022", UI_Graph_ADD, 2, UI_Color_Yellow, 1, (960+55), (540-130), (960+42), (540-227));
				Line_Draw(&right4to5, "023", UI_Graph_ADD, 2, UI_Color_Yellow, 1, (960+42), (540-227), (960+37), (540-320));
				
				//17mm turrent aim circle 2m, 3m, 5m
				Circle_Draw(&g17aim2m, "025", UI_Graph_ADD, 2, UI_Color_Purplish_red, 1,(960-72) , (540-114), 10 );//2m
				Circle_Draw(&g17aim3m, "026", UI_Graph_ADD, 2, UI_Color_Purplish_red, 1,(960-37) ,(540-116) , 8);//3m
				Circle_Draw(&g17aim5m, "027", UI_Graph_ADD, 2, UI_Color_Purplish_red, 1,(960-23), (540-150), 5);//5m
				//17mm ami circle connection lines
				Line_Draw(&g17aim2to3m, "028", UI_Graph_ADD, 2, UI_Color_Purplish_red, 2, (960-72), (540-114), (960-37), (540-116));
				Line_Draw(&g17aim3to5m, "029", UI_Graph_ADD, 2, UI_Color_Purplish_red, 1, (960-37), (540-116), (960-23), (540-150));
				//左边火控相关信息
				Char_Draw(&strCVSts, "012", UI_Graph_ADD, 2, UI_Color_Yellow, 20, 19, 3, CV_STS_X, CV_STS_Y,                              			   "CV:  OFF  AID  LOCK");  
				Char_Draw(&strGunSts, "013", UI_Graph_ADD, 2, UI_Color_Yellow, 20, 19, 3, GUN_STS_X, GUN_STS_Y,                           			   "GUN: OFF  SEMI AUTO");  
				Char_Draw(&strABoxSts, "014", UI_Graph_ADD, 2, UI_Color_Yellow, 20, 14, 3, AmmoBox_cover_STS_X, AmmoBox_cover_STS_Y,      			   "ABC: OFF  OPEN");
				Char_Draw(&strProjSLimSts, "015", UI_Graph_ADD, 2, UI_Color_Yellow, 20, 8, 3, Enemy_dis_STS_X, Enemy_dis_STS_Y, 									 "DS:    m");
				Char_Draw(&strDisSts, "016", UI_Graph_ADD, 2, UI_Color_Yellow, 20, 10, 3, Projectile_speed_lim_STS_X, Projectile_speed_lim_STS_Y,  "PL:    m/s");
				

				//更新 同态图标的坐标数据------------
				ui_coord_update();
				//动态图层 占用图层 4,5,6,7 ****在这里加/改了东西之后记得要在ui_dynamic_crt_send_fuc() 里也加/改****
				//右上角 超级电容电压
				//离线时superCap_info.VBKelvin_fromCap修改为 0
			 // Float_Draw(&fCapVolt, "999", UI_Graph_Change, 4, UI_Color_Main, 20, 2, 3, 1620, 810, ui_info.cap_volt);
				Float_Draw(&fCapVolt, "999", UI_Graph_Change, 4, UI_Color_Main, 20, 2, 3, 960, 200-50, ui_info.cap_volt);

				//右上角 超级电容百分比
				//离线时superCap_info.EBPct_fromCap修改为 0
			//	Float_Draw(&fCapPct, "998", UI_Graph_Change, 4, UI_Color_Main, 20, 2, 3, 1620, 840, ui_info.cap_pct);
				Float_Draw(&fCapPct, "998", UI_Graph_Change, 4, UI_Color_Main, 40, 2, 3, 960, 250-50, ui_info.cap_pct);

				//右上角方框
				Rectangle_Draw(&gChassisSts_box, "997", UI_Graph_Change, 4, UI_Color_Cyan, 3, ui_info.box_chassis_sts_coord[0]-80, ui_info.box_chassis_sts_coord[1]-95, ui_info.box_chassis_sts_coord[2]-80, ui_info.box_chassis_sts_coord[3]-95);
				Rectangle_Draw(&gSPINSts_box, "996", UI_Graph_Change, 4, UI_Color_Cyan, 3, ui_info.box_spin_sts_coord[0]-80, ui_info.box_spin_sts_coord[1]-95, ui_info.box_spin_sts_coord[2]-80, ui_info.box_spin_sts_coord[3]-95);
				
				//左上角---还未改
				Rectangle_Draw(&gCVSts_box, "995", UI_Graph_Change, 4, UI_Color_Cyan, 3, ui_info.box_cv_sts_coord[0], ui_info.box_cv_sts_coord[1], ui_info.box_cv_sts_coord[2], ui_info.box_cv_sts_coord[3]);
				Rectangle_Draw(&gGunSts_box, "994", UI_Graph_Change, 4, UI_Color_Cyan, 3, ui_info.box_gun_sts_coord[0], ui_info.box_gun_sts_coord[1], ui_info.box_gun_sts_coord[2], ui_info.box_gun_sts_coord[3]);
				Rectangle_Draw(&gABoxSts_box, "993", UI_Graph_Change, 4, UI_Color_Cyan, 3, ui_info.box_ammoBox_sts_coord[0], ui_info.box_ammoBox_sts_coord[1], ui_info.box_ammoBox_sts_coord[2], ui_info.box_ammoBox_sts_coord[3]);
				
				
				Float_Draw(&fProjSLim, "992", UI_Graph_Change, 4, UI_Color_Main, 20, 2, 3, 240, 720, ui_info.enemy_dis);
				Float_Draw(&fDis, "991", UI_Graph_Change, 4, UI_Color_Main, 20, 2, 3, 240, 680, ui_info.proj_speed_limit);
				
				//CV是否识别到目标
				if(miniPC_info.enemy_detected == 1)
				{
					Circle_Draw(&gEnemyDetected_circle, "990", UI_Graph_Change, 4, UI_Color_Green, 15, TopLeft_Cir_on_cv_DET_START_X, TopLeft_Cir_on_cv_DET_START_Y, TopLeft_Cir_on_cv_DET_radius);
				}
				else
				{
					Circle_Draw(&gEnemyDetected_circle, "990", UI_Graph_Change, 4, UI_Color_Cyan, 15, TopLeft_Cir_on_cv_DET_START_X, TopLeft_Cir_on_cv_DET_START_Y, TopLeft_Cir_on_cv_DET_radius);
				}
				
				Rectangle_Draw(&gCVfb_sts_box, "989", UI_Graph_Change, 4, UI_Color_White, 3, ui_info.box_cv_feedback_sts[0], ui_info.box_cv_feedback_sts[1], ui_info.box_cv_feedback_sts[2], ui_info.box_cv_feedback_sts[3]);
				
				//****在这里 上面 加/改了东西之后记得要在ui_dynamic_crt_send_fuc() 里也加/改****
				
				//完成绘制 开始发送 先发静态
				//refresh UI and String(Char)
				UI_ReFresh(1,gAimVertL);
				UI_ReFresh(5, gAimHorizL2m, gAimHorizL3m, gAimHorizL4m, gAimHorizL5m);
				UI_ReFresh(5, left2to3, left3to4,left4to5,right3to4, right2to3);
				UI_ReFresh(5, right4to5);
				UI_ReFresh(5,g17aim2m,g17aim3m);
				UI_ReFresh(5,g17aim5m);
				UI_ReFresh(5,g17aim2to3m,g17aim3to5m);//17mm aim circle connection lines
				//Right
				//Char_ReFresh(strCapVolt);
				//Char_ReFresh(strCapPct);	
				Char_ReFresh(strChassisSts);
				Char_ReFresh(strSPINSts);
				//Left
				Char_ReFresh(strCVSts);
				Char_ReFresh(strGunSts);
				Char_ReFresh(strABoxSts);
				Char_ReFresh(strProjSLimSts);
				Char_ReFresh(strDisSts);
				
				//动态的修改 发送
				UI_ReFresh(2, fCapVolt, fCapPct);
				UI_ReFresh(2, fProjSLim, fDis);
				UI_ReFresh(2, gEnemyDetected_circle, gCVfb_sts_box);
				//UI_ReFresh(5, fCapVolt, fCapPct, fProjSLim, fDis, gEnemyDetected_circle);
				UI_ReFresh(5, gChassisSts_box, gSPINSts_box, gCVSts_box, gGunSts_box, gABoxSts_box);
				
				//定时创建一次动态的--------------
				if(xTaskGetTickCount() - ui_dynamic_crt_sendFreq > ui_dynamic_crt_send_TimeStamp)
				{
						ui_dynamic_crt_send_TimeStamp = xTaskGetTickCount(); //更新时间戳 
						ui_dynamic_crt_send_fuc(); //到时间了, 在客户端创建一次动态的图像
				}
//				temp_time_check_RTOS = xTaskGetTickCount();
//			 temp_time_check_HAL = HAL_GetTick();
//			 _temp_a++;
			 
				vTaskDelay(100);
				//
				client_ui_count_ref++;
				
				if(client_ui_count_ref % 10 == 0)
					client_ui_test_flag = -1 * client_ui_test_flag;
				
			
#if INCLUDE_uxTaskGetStackHighWaterMark
        client_ui_task_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif	
		}
}

/*通过判断相关 状态机 -> ui坐标 */
void ui_coord_update()
{
	 //底盘 turbo 模式的判断
	 if(turboMode == 0)
	 {
		 ui_info.ui_chassis_sts = NORM;
		 ui_info.box_chassis_sts_coord[0] = TopRight_REC_on_NORM_START_X;
		 ui_info.box_chassis_sts_coord[1] = TopRight_REC_on_NORM_START_Y;
		 ui_info.box_chassis_sts_coord[2] = TopRight_REC_on_NORM_END_X;
		 ui_info.box_chassis_sts_coord[3] = TopRight_REC_on_NORM_END_Y;
	 }
	 else
	 {
		 ui_info.ui_chassis_sts = BOOST;
		 ui_info.box_chassis_sts_coord[0] = TopRight_REC_on_BOOST_START_X;
		 ui_info.box_chassis_sts_coord[1] = TopRight_REC_on_BOOST_START_Y;
		 ui_info.box_chassis_sts_coord[2] = TopRight_REC_on_BOOST_END_X;
		 ui_info.box_chassis_sts_coord[3] = TopRight_REC_on_BOOST_END_Y;
	 }
	 
	 /*
		swing_flag是小陀螺状态机
		swing_flag = 0 无小陀螺
		swing_flag = 1 有小陀螺
		*/
	 if(swing_flag == 0)
	 {
		 ui_info.ui_spin_sts = spinFOLL;
		 ui_info.box_spin_sts_coord[0] = TopRight_REC_on_FOLL_START_X;
		 ui_info.box_spin_sts_coord[1] = TopRight_REC_on_FOLL_START_Y;
		 ui_info.box_spin_sts_coord[2] = TopRight_REC_on_FOLL_END_X;
		 ui_info.box_spin_sts_coord[3] = TopRight_REC_on_FOLL_END_Y;
	 }
	 else
	 {
		 ui_info.ui_spin_sts = spinSPIN;
		 ui_info.box_spin_sts_coord[0] = TopRight_REC_on_SPIN_START_X;
		 ui_info.box_spin_sts_coord[1] = TopRight_REC_on_SPIN_START_Y;
		 ui_info.box_spin_sts_coord[2] = TopRight_REC_on_SPIN_END_X;
		 ui_info.box_spin_sts_coord[3] = TopRight_REC_on_SPIN_END_Y;
	 }
	 
	 //CV 状态机 状态 //自动瞄准开关状态 0关 1自动瞄准
	 if(miniPC_info.autoAimFlag == 0)
	 {
		 ui_info.ui_cv_sts = cvOFF;
		 ui_info.box_cv_sts_coord[0] = TopLeft_REC_on_cv_OFF_START_X;
		 ui_info.box_cv_sts_coord[1] = TopLeft_REC_on_cv_OFF_START_Y;
		 ui_info.box_cv_sts_coord[2] = TopLeft_REC_on_cv_OFF_END_X;
		 ui_info.box_cv_sts_coord[3] = TopLeft_REC_on_cv_OFF_END_Y;
	 }
	 else if(miniPC_info.autoAimFlag == 1)
	 {
		 ui_info.ui_cv_sts = cvAID;
		 ui_info.box_cv_sts_coord[0] = TopLeft_REC_on_cv_AID_START_X;
		 ui_info.box_cv_sts_coord[1] = TopLeft_REC_on_cv_AID_START_Y;
		 ui_info.box_cv_sts_coord[2] = TopLeft_REC_on_cv_AID_END_X;
		 ui_info.box_cv_sts_coord[3] = TopLeft_REC_on_cv_AID_END_Y;
	 }
	 else if(miniPC_info.autoAimFlag == 2)
	 {
		 ui_info.ui_cv_sts = cvLOCK;
		 ui_info.box_cv_sts_coord[0] = TopLeft_REC_on_cv_LOCK_START_X;
		 ui_info.box_cv_sts_coord[1] = TopLeft_REC_on_cv_LOCK_START_Y;
		 ui_info.box_cv_sts_coord[2] = TopLeft_REC_on_cv_LOCK_END_X;
		 ui_info.box_cv_sts_coord[3] = TopLeft_REC_on_cv_LOCK_END_Y;
	 }
	 
	 //CV feedback 状态机
	 if(miniPC_info.cv_status == 0)
	 {
		 ui_info.ui_cv_feedback_sts = cvOFF;
		 ui_info.box_cv_feedback_sts[0] = TopLeft_CV_FEEDBACK_STATUS_on_OFF_START_X;
		 ui_info.box_cv_feedback_sts[1] = TopLeft_CV_FEEDBACK_STATUS_on_OFF_START_Y;
		 ui_info.box_cv_feedback_sts[2] = TopLeft_CV_FEEDBACK_STATUS_on_OFF_END_X;
		 ui_info.box_cv_feedback_sts[3] = TopLeft_CV_FEEDBACK_STATUS_on_OFF_END_Y;
	 }
	 else if(miniPC_info.cv_status == 1)
	 {
		 ui_info.ui_cv_feedback_sts = cvAID;
		 ui_info.box_cv_feedback_sts[0] = TopLeft_CV_FEEDBACK_STATUS_on_AID_START_X;
		 ui_info.box_cv_feedback_sts[1] = TopLeft_CV_FEEDBACK_STATUS_on_AID_START_Y;
		 ui_info.box_cv_feedback_sts[2] = TopLeft_CV_FEEDBACK_STATUS_on_AID_END_X;
		 ui_info.box_cv_feedback_sts[3] = TopLeft_CV_FEEDBACK_STATUS_on_AID_END_Y;
	 }
	 else if(miniPC_info.cv_status == 2)
	 {
		 ui_info.ui_cv_feedback_sts = cvLOCK;
		 ui_info.box_cv_feedback_sts[0] = TopLeft_CV_FEEDBACK_STATUS_on_LOCK_START_X;
		 ui_info.box_cv_feedback_sts[1] = TopLeft_CV_FEEDBACK_STATUS_on_LOCK_START_Y;
		 ui_info.box_cv_feedback_sts[2] = TopLeft_CV_FEEDBACK_STATUS_on_LOCK_END_X;
		 ui_info.box_cv_feedback_sts[3] = TopLeft_CV_FEEDBACK_STATUS_on_LOCK_END_Y;
	 }
	 
	 //CV状态机掉线提示
	 if(toe_is_error(MINIPC_TOE))
	 {
		 ui_info.ui_cv_feedback_sts = cvOFF;
		 ui_info.box_cv_feedback_sts[0] = TopLeft_CV_FEEDBACK_STATUS_on_OFF_START_X;
		 ui_info.box_cv_feedback_sts[1] = TopLeft_CV_FEEDBACK_STATUS_on_OFF_START_Y;
		 ui_info.box_cv_feedback_sts[2] = TopLeft_CV_FEEDBACK_STATUS_on_OFF_END_X;
		 ui_info.box_cv_feedback_sts[3] = TopLeft_CV_FEEDBACK_STATUS_on_OFF_END_Y;
	 }
	 
	 //GUN 状态机 状态
	 if(shoot_control.shoot_mode == SHOOT_STOP)
	 {
		 ui_info.ui_gun_sts = gunOFF;
		 ui_info.box_gun_sts_coord[0] = TopLeft_REC_on_gun_OFF_START_X;
		 ui_info.box_gun_sts_coord[1] = TopLeft_REC_on_gun_OFF_START_Y;
		 ui_info.box_gun_sts_coord[2] = TopLeft_REC_on_gun_OFF_END_X;
		 ui_info.box_gun_sts_coord[3] = TopLeft_REC_on_gun_OFF_END_Y;
	 }
	 else if(shoot_control.user_fire_ctrl == user_SHOOT_AUTO)
	 {
		 ui_info.ui_gun_sts = gunAUTO;
		 ui_info.box_gun_sts_coord[0] = TopLeft_REC_on_gun_AUTO_START_X;
		 ui_info.box_gun_sts_coord[1] = TopLeft_REC_on_gun_AUTO_START_Y;
		 ui_info.box_gun_sts_coord[2] = TopLeft_REC_on_gun_AUTO_END_X;
		 ui_info.box_gun_sts_coord[3] = TopLeft_REC_on_gun_AUTO_END_Y;
	 }
	 else if(shoot_control.user_fire_ctrl == user_SHOOT_SEMI)
	 {
		 ui_info.ui_gun_sts = gunSEMI;
		 ui_info.box_gun_sts_coord[0] = TopLeft_REC_on_gun_SEMI_START_X;
		 ui_info.box_gun_sts_coord[1] = TopLeft_REC_on_gun_SEMI_START_Y;
		 ui_info.box_gun_sts_coord[2] = TopLeft_REC_on_gun_SEMI_END_X;
		 ui_info.box_gun_sts_coord[3] = TopLeft_REC_on_gun_SEMI_END_Y;
	 }
	 
	 //Ammo Box 状态机 状态
	 if(shoot_control.ammoBox_sts == ammoOFF)
	 {
		 ui_info.ui_ammoBox_sts = ammoOFF;
		 //ui_info.box_ammoBox_sts_coord
		 ui_info.box_ammoBox_sts_coord[0] = TopLeft_REC_on_ammo_OFF_START_X;
		 ui_info.box_ammoBox_sts_coord[1] = TopLeft_REC_on_ammo_OFF_START_Y;
		 ui_info.box_ammoBox_sts_coord[2] = TopLeft_REC_on_ammo_OFF_END_X;
		 ui_info.box_ammoBox_sts_coord[3] = TopLeft_REC_on_ammo_OFF_END_Y;
	 }
	 else if(shoot_control.ammoBox_sts == ammoOPEN)
	 {
		 ui_info.ui_ammoBox_sts = ammoOPEN;
		 //ui_info.box_ammoBox_sts_coord
		 ui_info.box_ammoBox_sts_coord[0] = TopLeft_REC_on_ammo_OPEN_START_X;
		 ui_info.box_ammoBox_sts_coord[1] = TopLeft_REC_on_ammo_OPEN_START_Y;
		 ui_info.box_ammoBox_sts_coord[2] = TopLeft_REC_on_ammo_OPEN_END_X;
		 ui_info.box_ammoBox_sts_coord[3] = TopLeft_REC_on_ammo_OPEN_END_Y;
	 }

	 //开始整数字相关的东西 即插即用的超级电容控制板 判断
	 if(current_superCap == SuperCap_ID)
	 {
		 if(toe_is_error(SUPERCAP_TOE))
		 {
			 ui_info.cap_pct = 0.0f;
			 ui_info.cap_volt = 0.0f;
		 }
		 else
		 {
			 ui_info.cap_pct = superCap_info.EBPct_fromCap;
			 ui_info.cap_volt = superCap_info.VBKelvin_fromCap;
		 }
	 }
	 else
	 {
		 if(toe_is_error(WULIE_CAP_TOE))
		 {
			 ui_info.cap_pct = 0.0f;
			 ui_info.cap_volt = 0.0f;
		 }
		 else
		 {
			 ui_info.cap_pct = wulie_Cap_info.EBPct;
		   ui_info.cap_volt = wulie_Cap_info.cap_voltage;
		 }
	 }
	 
	 ui_info.enemy_dis = miniPC_info.dis;//和张哥商量
	 ui_info.proj_speed_limit = get_shooter_id1_17mm_speed_limit();
}

void ui_dynamic_crt_send_fuc()
{
		//右上角 超级电容电压
		//离线时superCap_info.VBKelvin_fromCap修改为 0
//		Float_Draw(&fCapVolt, "999", UI_Graph_ADD, 4, UI_Color_Main, 20, 2, 3, 1620, 810, ui_info.cap_volt);
				Float_Draw(&fCapVolt, "999", UI_Graph_ADD, 4, UI_Color_Main, 20, 2, 3, 960, 200-50, ui_info.cap_volt);

		//右上角 超级电容百分比
		//离线时superCap_info.EBPct_fromCap修改为 0
//		Float_Draw(&fCapPct, "998", UI_Graph_ADD, 4, UI_Color_Main, 20, 2, 3, 1620, 840, ui_info.cap_pct);
		Float_Draw(&fCapPct, "998", UI_Graph_ADD, 4, UI_Color_Main, 40, 2, 3, 960, 250-50, ui_info.cap_pct);
		
		//右上角方框
		Rectangle_Draw(&gChassisSts_box, "997", UI_Graph_ADD, 4, UI_Color_Cyan, 3, ui_info.box_chassis_sts_coord[0]-80, ui_info.box_chassis_sts_coord[1]-95, ui_info.box_chassis_sts_coord[2]-80, ui_info.box_chassis_sts_coord[3]-95);
		Rectangle_Draw(&gSPINSts_box, "996", UI_Graph_ADD, 4, UI_Color_Cyan, 3, ui_info.box_spin_sts_coord[0]-80, ui_info.box_spin_sts_coord[1]-95, ui_info.box_spin_sts_coord[2]-80, ui_info.box_spin_sts_coord[3]-95);
				
		//左上角---还未改
		Rectangle_Draw(&gCVSts_box, "995", UI_Graph_ADD, 4, UI_Color_Cyan, 3, ui_info.box_cv_sts_coord[0], ui_info.box_cv_sts_coord[1], ui_info.box_cv_sts_coord[2], ui_info.box_cv_sts_coord[3]);
		Rectangle_Draw(&gGunSts_box, "994", UI_Graph_ADD, 4, UI_Color_Cyan, 3, ui_info.box_gun_sts_coord[0], ui_info.box_gun_sts_coord[1], ui_info.box_gun_sts_coord[2], ui_info.box_gun_sts_coord[3]);
		Rectangle_Draw(&gABoxSts_box, "993", UI_Graph_ADD, 4, UI_Color_Cyan, 3, ui_info.box_ammoBox_sts_coord[0], ui_info.box_ammoBox_sts_coord[1], ui_info.box_ammoBox_sts_coord[2], ui_info.box_ammoBox_sts_coord[3]);
		
		Float_Draw(&fProjSLim, "992", UI_Graph_ADD, 4, UI_Color_Main, 20, 2, 3, 240, 720, ui_info.proj_speed_limit);
		Float_Draw(&fDis, "991", UI_Graph_ADD, 4, UI_Color_Main, 20, 2, 3, 240, 680, ui_info.enemy_dis);
	
		//CV是否识别到目标
		if(miniPC_info.enemy_detected == 1)
		{
			Circle_Draw(&gEnemyDetected_circle, "990", UI_Graph_ADD, 4, UI_Color_Green, 15, TopLeft_Cir_on_cv_DET_START_X, TopLeft_Cir_on_cv_DET_START_Y, TopLeft_Cir_on_cv_DET_radius);
		}
		else
		{
			Circle_Draw(&gEnemyDetected_circle, "990", UI_Graph_ADD, 4, UI_Color_Cyan, 15, TopLeft_Cir_on_cv_DET_START_X, TopLeft_Cir_on_cv_DET_START_Y, TopLeft_Cir_on_cv_DET_radius);
		}
		
		Rectangle_Draw(&gCVfb_sts_box, "989", UI_Graph_ADD, 4, UI_Color_White, 3, ui_info.box_cv_feedback_sts[0], ui_info.box_cv_feedback_sts[1], ui_info.box_cv_feedback_sts[2], ui_info.box_cv_feedback_sts[3]);
				
		//新增绘制结束		
				
		//动态的修改 发送
		UI_ReFresh(2, fCapVolt, fCapPct);
		UI_ReFresh(2, fProjSLim, fDis);
		UI_ReFresh(2, gEnemyDetected_circle, gCVfb_sts_box);
		UI_ReFresh(5, gChassisSts_box, gSPINSts_box, gCVSts_box, gGunSts_box, gABoxSts_box);
}

////先开始删除 图层4 5 6 7
//				delLayer.Delete_Operate = UI_Data_Del_Layer;
//				delLayer.Layer = 4;
//				Delete_ReFresh(delLayer);
//			
//			  delLayer.Layer = 5;
//				Delete_ReFresh(delLayer);
//			
//			  delLayer.Layer = 6;
//				Delete_ReFresh(delLayer);
//			
//			  delLayer.Layer = 7;
//				Delete_ReFresh(delLayer);


/****************************************串口驱动映射************************************/
void UI_SendByte(unsigned char ch)
{
//   USART_SendData(USART3,ch);
//   while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);	
	HAL_UART_Transmit(&huart6, (uint8_t*)&ch, 1,999);
//	while(HAL_UART_GetState(&huart6) == HAL_UART_STATE_BUSY_TX)
//	{
//		vTaskDelay(1);
//	}
//	HAL_UART_Transmit_IT(&huart6, (uint8_t*)&ch, 1);
	//while(!(huart6.Instance->SR))
//	while(!(__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TC) == 1))
//	{
//		vTaskDelay(1);
//	}
}

/********************************************删除操作*************************************
**参数：Del_Operate  对应头文件删除操作
        Del_Layer    要删除的层 取值0-9
*****************************************************************************************/

void UI_Delete(u8 Del_Operate,u8 Del_Layer)
{

   unsigned char *framepoint;                      //读写指针
   u16 frametail=0xFFFF;                        //CRC16校验值
   int loop_control;                       //For函数循环控制
   
   UI_Packhead framehead;
   UI_Data_Operate datahead;
   UI_Data_Delete del;
   
   framepoint=(unsigned char *)&framehead;
   
   framehead.SOF=UI_SOF;
   framehead.Data_Length=8;
   framehead.Seq=UI_Seq;
   framehead.CRC8=Get_CRC8_Check_Sum_UI(framepoint,4,0xFF);
   framehead.CMD_ID=UI_CMD_Robo_Exchange;                   //填充包头数据
   
   datahead.Data_ID=UI_Data_ID_Del;
	 /*SZL 6-16-2022 动态的收取 裁判系统ID*/
   datahead.Sender_ID=get_robot_id(); //Robot_ID;
	 switch(get_robot_id()){
		 case UI_Data_RobotID_RHero:
				datahead.Receiver_ID = UI_Data_CilentID_RHero;//为英雄操作手客户端(红)
				break;
			case UI_Data_RobotID_REngineer:
				datahead.Receiver_ID = UI_Data_CilentID_REngineer;
				break;
			case UI_Data_RobotID_RStandard1:
				datahead.Receiver_ID = UI_Data_CilentID_RStandard1;
				break;
			case UI_Data_RobotID_RStandard2:
				datahead.Receiver_ID = UI_Data_CilentID_RStandard2;
				break;
			case UI_Data_RobotID_RStandard3:
				datahead.Receiver_ID = UI_Data_CilentID_RStandard3;
				break;
			case UI_Data_RobotID_RAerial:
				datahead.Receiver_ID = UI_Data_CilentID_RAerial;
				break;
			
			case UI_Data_RobotID_BHero:
				datahead.Receiver_ID = UI_Data_CilentID_BHero;
				break;
			case UI_Data_RobotID_BEngineer:
				datahead.Receiver_ID = UI_Data_CilentID_BEngineer;
				break;
			case UI_Data_RobotID_BStandard1:
				datahead.Receiver_ID = UI_Data_CilentID_BStandard1;
				break;
			case UI_Data_RobotID_BStandard2:
				datahead.Receiver_ID = UI_Data_CilentID_BStandard2;
				break;
			case UI_Data_RobotID_BStandard3:
				datahead.Receiver_ID = UI_Data_CilentID_BStandard3;
				break;
			case UI_Data_RobotID_BAerial:
				datahead.Receiver_ID = UI_Data_CilentID_BAerial;
				break;
			default :
				datahead.Receiver_ID = Cilent_ID; //默认 无论怎样发一个出去
				datahead.Sender_ID = Robot_ID;
				break;
	 }//填充操作数据
   
   del.Delete_Operate=Del_Operate;
   del.Layer=Del_Layer;                                     //控制信息
   
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(framehead),frametail);
   framepoint=(unsigned char *)&datahead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(datahead),frametail);
   framepoint=(unsigned char *)&del;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(del),frametail);  //CRC16校验值计算
   
   framepoint=(unsigned char *)&framehead;
   for(loop_control=0;loop_control<sizeof(framehead);loop_control++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   }
   framepoint=(unsigned char *)&datahead;
   for(loop_control=0;loop_control<sizeof(datahead);loop_control++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   }
   framepoint=(unsigned char *)&del;
   for(loop_control=0;loop_control<sizeof(del);loop_control++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   }                                                                 //发送所有帧
   framepoint=(unsigned char *)&frametail;
   for(loop_control=0;loop_control<sizeof(frametail);loop_control++)
   {
      UI_SendByte(*framepoint);
      framepoint++;                                                  //发送CRC16校验值
   }
   
   UI_Seq++;                                                         //包序号+1
}
/************************************************绘制直线*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Start_x、Start_x    开始坐标
        End_x、End_y   结束坐标
**********************************************************************************************************/
        
void Line_Draw(Graph_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 End_x,u32 End_y)
{
   int i;
//   for(i=0;i<3&&imagename[i]!='\0';i++)
//      image->graphic_name[2-i]=imagename[i];
	
	 //SZL 6-8-2022修改 图形名按照写入数组的顺序存
	 for(i=0;i<3&&imagename[i]!='\0';i++)
      image->graphic_name[i]=imagename[i];
	
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->end_x = End_x;
   image->end_y = End_y;
}

/************************************************绘制矩形*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Start_x、Start_x    开始坐标
        End_x、End_y   结束坐标（对顶角坐标）
**********************************************************************************************************/
        
void Rectangle_Draw(Graph_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 End_x,u32 End_y)
{
   int i;
//   for(i=0;i<3&&imagename[i]!=\0;i++)
//      image->graphic_name[2-i]=imagename[i];
	
	  //SZL 6-8-2022修改 图形名按照写入数组的顺序存
	 for(i=0;i<3&&imagename[i]!='\0';i++)
      image->graphic_name[i]=imagename[i];
	
   image->graphic_tpye = UI_Graph_Rectangle;
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->end_x = End_x;
   image->end_y = End_y;
}

/************************************************绘制整圆*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Start_x、Start_x    圆心坐标
        Graph_Radius  图形半径
**********************************************************************************************************/
        
void Circle_Draw(Graph_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 Graph_Radius)
{
   int i;
//   for(i=0;i<3&&imagename[i]!=\0;i++)
//      image->graphic_name[2-i]=imagename[i];
	
	  //SZL 6-8-2022修改 图形名按照写入数组的顺序存
	 for(i=0;i<3&&imagename[i]!='\0';i++)
      image->graphic_name[i]=imagename[i];
	
   image->graphic_tpye = UI_Graph_Circle;
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->radius = Graph_Radius;
}

/************************************************绘制圆弧*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Graph_StartAngle,Graph_EndAngle    开始，终止角度
        Start_y,Start_y    圆心坐标
        x_Length,y_Length   x,y方向上轴长，参考椭圆
**********************************************************************************************************/
        
void Arc_Draw(Graph_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_StartAngle,u32 Graph_EndAngle,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 x_Length,u32 y_Length)
{
   int i;
   
//   for(i=0;i<3&&imagename[i]!=\0;i++)
//      image->graphic_name[2-i]=imagename[i];
	
	  //SZL 6-8-2022修改 图形名按照写入数组的顺序存
	 for(i=0;i<3&&imagename[i]!='\0';i++)
      image->graphic_name[i]=imagename[i];
	
   image->graphic_tpye = UI_Graph_Arc;
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->start_angle = Graph_StartAngle;
   image->end_angle = Graph_EndAngle;
   image->end_x = x_Length;
   image->end_y = y_Length;
}



/************************************************绘制浮点型数据*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Graph_Size     字号
        Graph_Digit    小数位数
        Start_x、Start_x    开始坐标
        Graph_Float   要显示的变量
**********************************************************************************************************/
        
void Float_Draw(Float_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Size,u32 Graph_Digit,u32 Graph_Width,u32 Start_x,u32 Start_y,float Graph_Float)
{
   int i;
   
//   for(i=0;i<3&&imagename[i]!=\0;i++)
//      image->graphic_name[2-i]=imagename[i];
	
	  //SZL 6-8-2022修改 图形名按照写入数组的顺序存
	 for(i=0;i<3&&imagename[i]!='\0';i++)
      image->graphic_name[i]=imagename[i];
	
   image->graphic_tpye = UI_Graph_Float;
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->start_angle = Graph_Size;
   image->end_angle = Graph_Digit;
   image->graph_Float = (int32_t)(1000*Graph_Float);
	//SZL 6-8-2022 修改float类型协议
}



/************************************************绘制字符型数据*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Graph_Size     字号
        Graph_Digit    字符个数
        Start_x、Start_x    开始坐标
        *Char_Data          待发送字符串开始地址
**********************************************************************************************************/
        
void Char_Draw(String_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Size,u32 Graph_Digit,u32 Graph_Width,u32 Start_x,u32 Start_y,char *Char_Data)
{
   int i;
   
//   for(i=0;i<3&&imagename[i]!=\0;i++)
//      image->Graph_Control.graphic_name[2-i]=imagename[i];
	
	  //SZL 6-8-2022修改 图形名按照写入数组的顺序存
	 for(i=0;i<3&&imagename[i]!='\0';i++)
      image->Graph_Control.graphic_name[i]=imagename[i];
	
   image->Graph_Control.graphic_tpye = UI_Graph_Char;
   image->Graph_Control.operate_tpye = Graph_Operate;
   image->Graph_Control.layer = Graph_Layer;
   image->Graph_Control.color = Graph_Color;
   image->Graph_Control.width = Graph_Width;
   image->Graph_Control.start_x = Start_x;
   image->Graph_Control.start_y = Start_y;
   image->Graph_Control.start_angle = Graph_Size;
   image->Graph_Control.end_angle = Graph_Digit;
   
   for(i=0;i<Graph_Digit;i++)
   {
      image->show_Data[i]=*Char_Data;
      Char_Data++;
   }
}

/************************************************UI推送函数（使更改生效）*********************************
**参数： cnt   图形个数
         ...   图形变量参数


Tips：：该函数只能推送1，2，5，7个图形，其他数目协议未涉及
**********************************************************************************************************/
int UI_ReFresh(int cnt,...)
{
   int i,n;
   Graph_Data imageData;
   unsigned char *framepoint;                      //读写指针
   u16 frametail=0xFFFF;                        //CRC16校验值
   
   UI_Packhead framehead;//frame_header(5-byte)+cmd_id(2-byte)
   UI_Data_Operate datahead;//交互数据包括了一个统一的数据段头结构: 内容ID 发送者 接收者的ID和内容数据段
   
   va_list ap;
   va_start(ap,cnt);
   
   framepoint=(unsigned char *)&framehead;
   framehead.SOF=UI_SOF;
   framehead.Data_Length=6+cnt*15;
   framehead.Seq=UI_Seq;
   framehead.CRC8=Get_CRC8_Check_Sum_UI(framepoint,4,0xFF);
   framehead.CMD_ID=UI_CMD_Robo_Exchange;                   //填充包头数据
   
   switch(cnt)
   {
      case 1:
         datahead.Data_ID=UI_Data_ID_Draw1;
         break;
      case 2:
         datahead.Data_ID=UI_Data_ID_Draw2;
         break;
      case 5:
         datahead.Data_ID=UI_Data_ID_Draw5;
         break;
      case 7:
         datahead.Data_ID=UI_Data_ID_Draw7;
         break;
      default:
         return (-1);
   }
	 
   /*SZL 6-16-2022 动态的收取 裁判系统ID*/
   datahead.Sender_ID=get_robot_id(); //Robot_ID;
	 switch(get_robot_id()){
		 case UI_Data_RobotID_RHero:
				datahead.Receiver_ID = UI_Data_CilentID_RHero;//为英雄操作手客户端(红)
				break;
			case UI_Data_RobotID_REngineer:
				datahead.Receiver_ID = UI_Data_CilentID_REngineer;
				break;
			case UI_Data_RobotID_RStandard1:
				datahead.Receiver_ID = UI_Data_CilentID_RStandard1;
				break;
			case UI_Data_RobotID_RStandard2:
				datahead.Receiver_ID = UI_Data_CilentID_RStandard2;
				break;
			case UI_Data_RobotID_RStandard3:
				datahead.Receiver_ID = UI_Data_CilentID_RStandard3;
				break;
			case UI_Data_RobotID_RAerial:
				datahead.Receiver_ID = UI_Data_CilentID_RAerial;
				break;
			
			case UI_Data_RobotID_BHero:
				datahead.Receiver_ID = UI_Data_CilentID_BHero;
				break;
			case UI_Data_RobotID_BEngineer:
				datahead.Receiver_ID = UI_Data_CilentID_BEngineer;
				break;
			case UI_Data_RobotID_BStandard1:
				datahead.Receiver_ID = UI_Data_CilentID_BStandard1;
				break;
			case UI_Data_RobotID_BStandard2:
				datahead.Receiver_ID = UI_Data_CilentID_BStandard2;
				break;
			case UI_Data_RobotID_BStandard3:
				datahead.Receiver_ID = UI_Data_CilentID_BStandard3;
				break;
			case UI_Data_RobotID_BAerial:
				datahead.Receiver_ID = UI_Data_CilentID_BAerial;
				break;
			default :
				datahead.Receiver_ID = Cilent_ID; //默认 无论怎样发一个出去
				datahead.Sender_ID = Robot_ID;
				break;
	 }//填充操作数据
   
   framepoint=(unsigned char *)&framehead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(framehead),frametail);
   framepoint=(unsigned char *)&datahead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(datahead),frametail);          //CRC16校验值计算（部分）
   
   framepoint=(unsigned char *)&framehead;
   for(i=0;i<sizeof(framehead);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   }
   framepoint=(unsigned char *)&datahead;
   for(i=0;i<sizeof(datahead);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   }
   
   for(i=0;i<cnt;i++)
   {
      imageData=va_arg(ap,Graph_Data);
      
      framepoint=(unsigned char *)&imageData;
      frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(imageData),frametail);             //CRC16校验
      
      for(n=0;n<sizeof(imageData);n++)
      {
         UI_SendByte(*framepoint);
         framepoint++;             
      } //发送图片帧
   }
   framepoint=(unsigned char *)&frametail;
   for(i=0;i<sizeof(frametail);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;                                                  //发送CRC16校验值
   }
   
   va_end(ap);
   
   UI_Seq++;                                                         //包序号+1
   return 0;
}


/************************************************UI推送字符（使更改生效）*********************************
**参数： cnt   图形个数
         ...   图形变量参数


Tips：：该函数只能推送1，2，5，7个图形，其他数目协议未涉及
**********************************************************************************************************/
int Char_ReFresh(String_Data string_Data)
{
   int i;
   String_Data imageData;
   unsigned char *framepoint;                      //读写指针
   u16 frametail=0xFFFF;                        //CRC16校验值
   
   UI_Packhead framehead;//frame_header(5-byte)+cmd_id(2-byte)
   UI_Data_Operate datahead;//交互数据包括了一个统一的数据段头结构: 内容ID 发送者 接收者的ID和内容数据段
   imageData=string_Data;
   
   
   framepoint=(unsigned char *)&framehead;
   framehead.SOF=UI_SOF;
   framehead.Data_Length=6+45;
   framehead.Seq=UI_Seq;
   framehead.CRC8=Get_CRC8_Check_Sum_UI(framepoint,4,0xFF);
   framehead.CMD_ID=UI_CMD_Robo_Exchange;                   //填充包头数据
   

   datahead.Data_ID=UI_Data_ID_DrawChar;//SZL 6-8-2022 修改

   /*SZL 6-16-2022 动态的收取 裁判系统ID*/
   datahead.Sender_ID=get_robot_id(); //Robot_ID;
	 switch(get_robot_id()){
		 case UI_Data_RobotID_RHero:
				datahead.Receiver_ID = UI_Data_CilentID_RHero;//为英雄操作手客户端(红)
				break;
			case UI_Data_RobotID_REngineer:
				datahead.Receiver_ID = UI_Data_CilentID_REngineer;
				break;
			case UI_Data_RobotID_RStandard1:
				datahead.Receiver_ID = UI_Data_CilentID_RStandard1;
				break;
			case UI_Data_RobotID_RStandard2:
				datahead.Receiver_ID = UI_Data_CilentID_RStandard2;
				break;
			case UI_Data_RobotID_RStandard3:
				datahead.Receiver_ID = UI_Data_CilentID_RStandard3;
				break;
			case UI_Data_RobotID_RAerial:
				datahead.Receiver_ID = UI_Data_CilentID_RAerial;
				break;
			
			case UI_Data_RobotID_BHero:
				datahead.Receiver_ID = UI_Data_CilentID_BHero;
				break;
			case UI_Data_RobotID_BEngineer:
				datahead.Receiver_ID = UI_Data_CilentID_BEngineer;
				break;
			case UI_Data_RobotID_BStandard1:
				datahead.Receiver_ID = UI_Data_CilentID_BStandard1;
				break;
			case UI_Data_RobotID_BStandard2:
				datahead.Receiver_ID = UI_Data_CilentID_BStandard2;
				break;
			case UI_Data_RobotID_BStandard3:
				datahead.Receiver_ID = UI_Data_CilentID_BStandard3;
				break;
			case UI_Data_RobotID_BAerial:
				datahead.Receiver_ID = UI_Data_CilentID_BAerial;
				break;
			default :
				datahead.Receiver_ID = Cilent_ID; //默认 无论怎样发一个出去
				datahead.Sender_ID = Robot_ID;
				break;
	 }//填充操作数据
   
   framepoint=(unsigned char *)&framehead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(framehead),frametail);
   framepoint=(unsigned char *)&datahead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(datahead),frametail);
   framepoint=(unsigned char *)&imageData;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(imageData),frametail);             //CRC16校验   //CRC16校验值计算（部分）
   
   framepoint=(unsigned char *)&framehead;
   for(i=0;i<sizeof(framehead);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   }
   framepoint=(unsigned char *)&datahead;
   for(i=0;i<sizeof(datahead);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   }                                                   //发送操作数据  
   framepoint=(unsigned char *)&imageData;
   for(i=0;i<sizeof(imageData);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;             
   }                                               //发送图片帧
   
   
   
   framepoint=(unsigned char *)&frametail;
   for(i=0;i<sizeof(frametail);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;                                                  //发送CRC16校验值
   }
   
   
   UI_Seq++;                                                         //包序号+1
   return 0;
}

/*

*/
int Delete_ReFresh(UI_Data_Delete delete_Data)
{
	 int i;
   UI_Data_Delete imageData;
   unsigned char *framepoint;                      //读写指针
   u16 frametail=0xFFFF;                        //CRC16校验值
   
   UI_Packhead framehead;//frame_header(5-byte)+cmd_id(2-byte)
   UI_Data_Operate datahead;//交互数据包括了一个统一的数据段头结构: 内容ID 发送者 接收者的ID和内容数据段
   imageData=delete_Data;
   
   
   framepoint=(unsigned char *)&framehead;
   framehead.SOF=UI_SOF;
   framehead.Data_Length=6+2;
   framehead.Seq=UI_Seq;
   framehead.CRC8=Get_CRC8_Check_Sum_UI(framepoint,4,0xFF);
   framehead.CMD_ID=UI_CMD_Robo_Exchange;                   //填充包头数据
   

   datahead.Data_ID=UI_Data_ID_Del;

   /*SZL 6-16-2022 动态的收取 裁判系统ID*/
   datahead.Sender_ID=get_robot_id(); //Robot_ID;
	 switch(get_robot_id()){
		 case UI_Data_RobotID_RHero:
				datahead.Receiver_ID = UI_Data_CilentID_RHero;//为英雄操作手客户端(红)
				break;
			case UI_Data_RobotID_REngineer:
				datahead.Receiver_ID = UI_Data_CilentID_REngineer;
				break;
			case UI_Data_RobotID_RStandard1:
				datahead.Receiver_ID = UI_Data_CilentID_RStandard1;
				break;
			case UI_Data_RobotID_RStandard2:
				datahead.Receiver_ID = UI_Data_CilentID_RStandard2;
				break;
			case UI_Data_RobotID_RStandard3:
				datahead.Receiver_ID = UI_Data_CilentID_RStandard3;
				break;
			case UI_Data_RobotID_RAerial:
				datahead.Receiver_ID = UI_Data_CilentID_RAerial;
				break;
			
			case UI_Data_RobotID_BHero:
				datahead.Receiver_ID = UI_Data_CilentID_BHero;
				break;
			case UI_Data_RobotID_BEngineer:
				datahead.Receiver_ID = UI_Data_CilentID_BEngineer;
				break;
			case UI_Data_RobotID_BStandard1:
				datahead.Receiver_ID = UI_Data_CilentID_BStandard1;
				break;
			case UI_Data_RobotID_BStandard2:
				datahead.Receiver_ID = UI_Data_CilentID_BStandard2;
				break;
			case UI_Data_RobotID_BStandard3:
				datahead.Receiver_ID = UI_Data_CilentID_BStandard3;
				break;
			case UI_Data_RobotID_BAerial:
				datahead.Receiver_ID = UI_Data_CilentID_BAerial;
				break;
			default :
				datahead.Receiver_ID = Cilent_ID; //默认 无论怎样发一个出去
				datahead.Sender_ID = Robot_ID;
				break;
	 }//填充操作数据
   
   framepoint=(unsigned char *)&framehead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(framehead),frametail);
   framepoint=(unsigned char *)&datahead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(datahead),frametail);
   framepoint=(unsigned char *)&imageData;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(imageData),frametail);             //CRC16校验   //CRC16校验值计算（部分）
   
   framepoint=(unsigned char *)&framehead;
   for(i=0;i<sizeof(framehead);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   }
   framepoint=(unsigned char *)&datahead;
   for(i=0;i<sizeof(datahead);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   }                                                   //发送操作数据  
   framepoint=(unsigned char *)&imageData;
   for(i=0;i<sizeof(imageData);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;             
   }                                               //发送图片帧
   

   framepoint=(unsigned char *)&frametail;
   for(i=0;i<sizeof(frametail);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;                                                  //发送CRC16校验值
   }
   
   
   UI_Seq++;                                                         //包序号+1
   return 0;
}

/*****************************************************CRC8校验值计算**********************************************/
const unsigned char CRC8_INIT_UI = 0xff; 
const unsigned char CRC8_TAB_UI[256] = 
{ 
	0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41, 
	0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc, 
	0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62, 
	0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff, 
	0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07, 
	0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a, 
	0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24, 
	0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9, 
	0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd, 
	0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50, 
	0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee, 
	0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73, 
	0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b, 
	0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16, 
	0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8, 
	0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35, 
};
unsigned char Get_CRC8_Check_Sum_UI(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8) 
{ 
	unsigned char ucIndex; 
	while (dwLength--) 
	{ 
		ucIndex = ucCRC8^(*pchMessage++); 
		ucCRC8 = CRC8_TAB_UI[ucIndex]; 
	} 
	return(ucCRC8); 
}

uint16_t CRC_INIT_UI = 0xffff; 
const uint16_t wCRC_Table_UI[256] = 
{ 
	0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 
	0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 
	0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e, 
	0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 
	0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd, 
	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5, 
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 
	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974, 
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb, 
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72, 
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 
	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738, 
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 
	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff, 
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 
	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5, 
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134, 
	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c, 
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 
	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb, 
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232, 
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1, 
	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9, 
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 
	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};
/* 
** Descriptions: CRC16 checksum function 
** Input: Data to check,Stream length, initialized checksum 
** Output: CRC checksum 
*/ 
uint16_t Get_CRC16_Check_Sum_UI(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC) 
{ 
	Uint8_t chData; 
	if (pchMessage == NULL) 
	{ 
		return 0xFFFF; 
	} 
	while(dwLength--) 
	{ 
		chData = *pchMessage++;
		(wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table_UI[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 
		0x00ff]; 
	} 
	return wCRC; 
}



