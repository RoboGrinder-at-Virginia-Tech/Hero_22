/*************************************************************

RM自定义UI协议       基于RM2020学生串口通信协议V1.3

UI坐标标定

弗吉尼亚理工 Virginia Tech; RoboGrinder

**************************************************************/

#ifndef __CLIENT_UI_COORDINATE_INFO__
#define __CLIENT_UI_COORDINATE_INFO__

/*坐标 标定*/
#define Center_X 960
#define	Center_Y 540
//右上角 字符串 起始坐标:
#define	TopRight_String_Start_X 1440
#define TopRight_String_Start_Y 840
#define TopRight_static_Y_offset 30

#define CAP_PCT_X TopRight_String_Start_X //1440
#define CAP_PCT_Y TopRight_String_Start_Y //840

#define CAP_VOLT_X TopRight_String_Start_X //1440
#define CAP_VOLT_Y (TopRight_String_Start_Y-TopRight_static_Y_offset) //810


//右上角 传递动态信息的 字符串
#define TopRight_dynamic_Y_offset 40
#define CHASSIS_STS_X TopRight_String_Start_X //1440
#define CHASSIS_STS_Y (CAP_VOLT_Y-TopRight_dynamic_Y_offset) //770
#define SPIN_STS_X TopRight_String_Start_X //1440
#define SPIN_STS_Y (CHASSIS_STS_Y-TopRight_dynamic_Y_offset) //730

//左上角起始坐标
#define TopLeft_String_Start_X 150
#define TopLeft_String_Start_Y 840

//左上角 传递动态信息的 字符串
#define TopLeft_dynamic_Y_offset 40
#define CV_STS_X TopLeft_String_Start_X //150
#define CV_STS_Y TopLeft_String_Start_Y //840

#define GUN_STS_X TopLeft_String_Start_X //150
#define GUN_STS_Y CV_STS_Y-TopLeft_dynamic_Y_offset //800

#define AmmoBox_cover_STS_X TopLeft_String_Start_X //150
#define AmmoBox_cover_STS_Y GUN_STS_Y-TopLeft_dynamic_Y_offset //760

//左上角 静态信息 字符串
#define Enemy_dis_STS_X TopLeft_String_Start_X //150
#define Enemy_dis_STS_Y AmmoBox_cover_STS_Y-TopLeft_dynamic_Y_offset //720

#define Projectile_speed_lim_STS_X TopLeft_String_Start_X //150
#define Projectile_speed_lim_STS_Y Enemy_dis_STS_Y-TopLeft_dynamic_Y_offset //680


//动态方框 的长和宽 左下角为0,0 向上为正, 向右为正
#define UI_REC_LENGTH 95
#define UI_REC_WIDTH 35

//右上角 动态方框 CHASSIS_STS
#define TopRight_REC_START_offset_X 10 //相比于 要框上去的 字符串的offset
#define TopRight_REC_START_offset_Y 10

//右上角方框起始 未使用
#define TopRight_BOX_START_X 1430
#define TopRight_BOX_START_Y 780
#define TopRight_BOX_END_X TopRight_BOX_START_X+UI_REC_LENGTH //1525
#define TopRight_BOX_END_Y TopRight_BOX_START_Y-UI_REC_WIDTH //745

#define TopRight_REC_on_NORM_START_X CHASSIS_STS_X-TopRight_REC_START_offset_X //1430
#define TopRight_REC_on_NORM_START_Y CHASSIS_STS_Y+TopRight_REC_START_offset_Y //780
#define TopRight_REC_on_NORM_END_X TopRight_REC_on_NORM_START_X+UI_REC_LENGTH //1525
#define TopRight_REC_on_NORM_END_Y TopRight_REC_on_NORM_START_Y-UI_REC_WIDTH //745

#define TopRight_REC_on_BOOST_START_X TopRight_REC_on_NORM_START_X+UI_REC_LENGTH+5 //1525+5 = 1530
#define TopRight_REC_on_BOOST_START_Y TopRight_REC_on_NORM_START_Y //780
#define TopRight_REC_on_BOOST_END_X TopRight_REC_on_BOOST_START_X+UI_REC_LENGTH //1625
#define TopRight_REC_on_BOOST_END_Y TopRight_REC_on_BOOST_START_Y-UI_REC_WIDTH //745

//右上角 动态方框 SPIN_STS
#define TopRight_REC_on_FOLL_START_X SPIN_STS_X-TopRight_REC_START_offset_X //1430
#define TopRight_REC_on_FOLL_START_Y SPIN_STS_Y+TopRight_REC_START_offset_Y //740
#define TopRight_REC_on_FOLL_END_X TopRight_REC_on_FOLL_START_X+UI_REC_LENGTH //1525
#define TopRight_REC_on_FOLL_END_Y TopRight_REC_on_FOLL_START_Y-UI_REC_WIDTH //705

#define TopRight_REC_on_SPIN_START_X TopRight_REC_on_FOLL_START_X+UI_REC_LENGTH+5 //1530
#define TopRight_REC_on_SPIN_START_Y TopRight_REC_on_FOLL_START_Y //740
#define TopRight_REC_on_SPIN_END_X TopRight_REC_on_SPIN_START_X+UI_REC_LENGTH //1625
#define TopRight_REC_on_SPIN_END_Y TopRight_REC_on_SPIN_START_Y-UI_REC_WIDTH //705

//最左上角的第一个动态方框 起始位置
#define TopLeft_BOX_START_X 240
#define TopLeft_BOX_START_Y 850
#define TopLeft_BOX_END_X TopLeft_BOX_START_X+UI_REC_LENGTH //335
#define TopLeft_BOX_END_Y TopLeft_BOX_START_Y-UI_REC_WIDTH //815


#define TopLeft_REC_START_offset_X 10 //相比于 要框上去的 字符串的offset 未用
#define TopLeft_REC_START_offset_Y 10
//左上角 动态方框 CVSts_box
#define TopLeft_REC_on_cv_OFF_START_X TopLeft_BOX_START_X //240
#define TopLeft_REC_on_cv_OFF_START_Y TopLeft_BOX_START_Y //850
#define TopLeft_REC_on_cv_OFF_END_X TopLeft_BOX_END_X //335
#define TopLeft_REC_on_cv_OFF_END_Y TopLeft_BOX_END_Y //815

#define TopLeft_REC_on_cv_AID_START_X TopLeft_REC_on_cv_OFF_START_X+UI_REC_LENGTH+5 //340
#define TopLeft_REC_on_cv_AID_START_Y TopLeft_REC_on_cv_OFF_START_Y //850
#define TopLeft_REC_on_cv_AID_END_X TopLeft_REC_on_cv_AID_START_X+UI_REC_LENGTH //435
#define TopLeft_REC_on_cv_AID_END_Y TopLeft_REC_on_cv_AID_START_Y-UI_REC_WIDTH //815

#define TopLeft_REC_on_cv_LOCK_START_X TopLeft_REC_on_cv_AID_START_X+UI_REC_LENGTH+5 //440
#define TopLeft_REC_on_cv_LOCK_START_Y TopLeft_REC_on_cv_OFF_START_Y //850
#define TopLeft_REC_on_cv_LOCK_END_X TopLeft_REC_on_cv_LOCK_START_X+UI_REC_LENGTH //535
#define TopLeft_REC_on_cv_LOCK_END_Y TopLeft_REC_on_cv_LOCK_START_Y-UI_REC_WIDTH //815
//
#define TopLeft_Cir_on_cv_DET_START_X 580
#define TopLeft_Cir_on_cv_DET_START_Y 832
#define TopLeft_Cir_on_cv_DET_radius 10

#define TopLeft_CV_FEEDBACK_STATUS_on_OFF_START_X TopLeft_REC_on_cv_OFF_START_X-8
#define TopLeft_CV_FEEDBACK_STATUS_on_OFF_START_Y TopLeft_REC_on_cv_OFF_START_Y+8
#define TopLeft_CV_FEEDBACK_STATUS_on_OFF_END_X TopLeft_REC_on_cv_OFF_END_X+8
#define TopLeft_CV_FEEDBACK_STATUS_on_OFF_END_Y TopLeft_REC_on_cv_OFF_END_Y-8

#define TopLeft_CV_FEEDBACK_STATUS_on_AID_START_X TopLeft_REC_on_cv_AID_START_X-8
#define TopLeft_CV_FEEDBACK_STATUS_on_AID_START_Y TopLeft_REC_on_cv_AID_START_Y+8
#define TopLeft_CV_FEEDBACK_STATUS_on_AID_END_X TopLeft_REC_on_cv_AID_END_X+8
#define TopLeft_CV_FEEDBACK_STATUS_on_AID_END_Y TopLeft_REC_on_cv_AID_END_Y-8

#define TopLeft_CV_FEEDBACK_STATUS_on_LOCK_START_X TopLeft_REC_on_cv_LOCK_START_X-8
#define TopLeft_CV_FEEDBACK_STATUS_on_LOCK_START_Y TopLeft_REC_on_cv_LOCK_START_Y+8
#define TopLeft_CV_FEEDBACK_STATUS_on_LOCK_END_X TopLeft_REC_on_cv_LOCK_END_X+8
#define TopLeft_CV_FEEDBACK_STATUS_on_LOCK_END_Y TopLeft_REC_on_cv_LOCK_END_Y-8

//左上角 动态方框 GUNSts_box
#define TopLeft_REC_on_gun_OFF_START_X TopLeft_REC_on_cv_OFF_START_X //240
#define TopLeft_REC_on_gun_OFF_START_Y TopLeft_REC_on_cv_OFF_START_Y-TopLeft_dynamic_Y_offset //810
#define TopLeft_REC_on_gun_OFF_END_X TopLeft_REC_on_gun_OFF_START_X+UI_REC_LENGTH //335
#define TopLeft_REC_on_gun_OFF_END_Y TopLeft_REC_on_gun_OFF_START_Y-UI_REC_WIDTH //810-35 = 775

#define TopLeft_REC_on_gun_SEMI_START_X TopLeft_REC_on_gun_OFF_START_X+UI_REC_LENGTH+5 //340
#define TopLeft_REC_on_gun_SEMI_START_Y TopLeft_REC_on_gun_OFF_START_Y //810
#define TopLeft_REC_on_gun_SEMI_END_X TopLeft_REC_on_gun_SEMI_START_X+UI_REC_LENGTH //435
#define TopLeft_REC_on_gun_SEMI_END_Y TopLeft_REC_on_gun_SEMI_START_Y-UI_REC_WIDTH //775

#define TopLeft_REC_on_gun_AUTO_START_X TopLeft_REC_on_gun_SEMI_START_X+UI_REC_LENGTH+5 //440
#define TopLeft_REC_on_gun_AUTO_START_Y TopLeft_REC_on_gun_OFF_START_Y //810
#define TopLeft_REC_on_gun_AUTO_END_X TopLeft_REC_on_gun_AUTO_START_X+UI_REC_LENGTH //535
#define TopLeft_REC_on_gun_AUTO_END_Y TopLeft_REC_on_gun_AUTO_START_Y-UI_REC_WIDTH //775

//左上角 动态方框 Ammo Box Cover Sts_box
#define TopLeft_REC_on_ammo_OFF_START_X TopLeft_REC_on_cv_OFF_START_X //240
#define TopLeft_REC_on_ammo_OFF_START_Y TopLeft_REC_on_gun_OFF_START_Y-TopLeft_dynamic_Y_offset //770
#define TopLeft_REC_on_ammo_OFF_END_X TopLeft_REC_on_ammo_OFF_START_X+UI_REC_LENGTH //335
#define TopLeft_REC_on_ammo_OFF_END_Y TopLeft_REC_on_ammo_OFF_START_Y-UI_REC_WIDTH //735

#define TopLeft_REC_on_ammo_OPEN_START_X TopLeft_REC_on_ammo_OFF_START_X+UI_REC_LENGTH+5 //340
#define TopLeft_REC_on_ammo_OPEN_START_Y TopLeft_REC_on_ammo_OFF_START_Y //770
#define TopLeft_REC_on_ammo_OPEN_END_X TopLeft_REC_on_ammo_OPEN_START_X+UI_REC_LENGTH //435
#define TopLeft_REC_on_ammo_OPEN_END_Y TopLeft_REC_on_ammo_OPEN_START_Y-UI_REC_WIDTH // 735

#endif //__CLIENT_UI_COORDINATE_INFO__

