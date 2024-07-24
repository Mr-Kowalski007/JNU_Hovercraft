#include "Control.h"
#define Motor_Enable_RVS Motor_Enable=Motor_Enable?0:1

/* 角度 */
float Roll_A0, Pitch_A0, Yaw_A0;
float Yaw_Lock_a;
float Pitch_a, Roll_a, Yaw_a;
float Pitch_a_Pi, Roll_a_Pi, Roll_error_Pi, Yaw_a_Pi, Yaw_a_Pi_genrl, Yaw_a_Pi_local;
float Pitch_a_last, Roll_a_last, Yaw_a_last;
float aim_roll_a,aim_pitch_a,aim_yaw_a,d_aim_yaw_a;
/* (角)速度 */
float Pitch_g, Roll_g, Yaw_g;
float Pitch_g_F, Roll_g_F, Yaw_g_F;
float Pitch_g_F_last, Roll_g_F_last;
float gyro_x_correction, gyro_y_correction, gyro_z_correction;//零漂修正
float Pitch_g_last, Roll_g_last, Yaw_g_last;
float aim_x_v,aim_y_v,aim_y_v_last,aim_yaw_g;
float X_V,Y_V;
float X_V_R_last,Y_V_R_last;
float X_V_last,Y_V_last;
float X_V_F,Y_V_F;
/* 加速度 */
float Pitch_acc, Roll_acc, Yaw_acc;
float Pitch_acc_F, Roll_acc_F, Yaw_acc_F;
float X_acc,Y_acc;

/* 位置 */
float Aim_X_Pos;
float X_Pos;
/*编码器计数*/
float Total_mileage; //总里程，前进为正
/* 速度 */
volatile float Aim_Speed, Now_Speed,Now_Speed_abs;
volatile sint32 PAW3395_X_SUM,PAW3395_Y_SUM;
uint8 Speed_Gear;
/* 标志位 */
uint8 Motor_Enable=0,Motor_Enable_last=1,ALLOW_RUNNING=0;
uint16 Motor_Enable_cnt=0,Motor_Disable_cnt=0;
uint8 Yaw_Lock, Yaw_Enable, Yaw_g_maintain, lean_correction;
sint8 Dynamic_Speed;
uint8 Dynamic_PIDK, Slope_Mode, RUSHING,Slope_Use_PV_Ki,PS2_MODE;

float   Force_FB=0,
        Force_LR=0,
        Force_Yaw=0,
        Force_0_FL,
        Force_1_FR,
        Force_2_BR,
        Force_3_BL;

uint16  Duty_Idle=1500, //MAX=10000
        Duty_Base=0,
        Duty_0_FL=0,
        Duty_1_FR=0,
        Duty_2_BR=0,
        Duty_3_BL=0,
        Duty_D_DO=0;

volatile PID_t PIDK_AA = {0,0,0},// X 平移
               PIDK_AB = {0,0,0},
               PIDK_AC = {0,0,0},

               PIDK_BA = {0,0,0},// Y 平移
               PIDK_BB = {0,0,0},
               PIDK_BC = {0,0,0},

               PIDK_YG = {0,0,0},// Yaw
               PIDK_YA = {0,0,0},
               PIDK_YV = {0,0,0},

               PIDK_CA = {0,0,0},
               PIDK_CB = {0,0,0},
               PIDK_CC = {0,0,0};

biquad_state X_V_biquad, Y_V_biquad,
             Pitch_g_biquad, Roll_g_biquad, Yaw_g_biquad,
             Pitch_acc_biquad, Roll_acc_biquad, Yaw_acc_biquad;

void Filter_Init(void){
    biquad_filter_init(&X_V_biquad, BIQUAD_LOWPASS, 1000, PIDK_AC.Kd, 0.7071);
    biquad_filter_init(&Y_V_biquad, BIQUAD_LOWPASS, 1000, PIDK_BC.Kd, 0.7071);
    biquad_filter_init(&Pitch_g_biquad, BIQUAD_LOWPASS, 4000, 30, 0.7071);
    biquad_filter_init(&Roll_g_biquad, BIQUAD_LOWPASS, 4000, 30, 0.7071);
    biquad_filter_init(&Yaw_g_biquad, BIQUAD_LOWPASS, 4000, 30, 0.7071);
    biquad_filter_init(&Pitch_acc_biquad, BIQUAD_LOWPASS, 4000, 10, 0.7071);
    biquad_filter_init(&Roll_acc_biquad, BIQUAD_LOWPASS, 4000, 10, 0.7071);
    biquad_filter_init(&Yaw_acc_biquad, BIQUAD_LOWPASS, 4000, 10, 0.7071);
}

#define TOTAL_MASS_gram_div_G (166.0f)
#define MASS_CENTER_HEIGHT (0.15f)
#define Gryo_to_MASS_CENTER_HEIGHT (0.15f)
void Control(void){
    static float PAW3395_V_K=1/(0.001*ENCODER_PER_METER_f),
                 PAW3395_M_K=Ang2Rad*MASS_CENTER_HEIGHT,
                 PAW3395_M_K1=Ang2Rad*Gryo_to_MASS_CENTER_HEIGHT,
                 PAW3395_MAX_dV=0.01,
                 Force_K=1/1.4142;
    static uint8 IRQCount;
    IRQCount++;

    if(Motor_Enable!=Motor_Enable_last){
        Filter_Init();
        Reset_Control();
        Motor_Enable_last=Motor_Enable;
    }

/* 读陀螺仪  */
    Get_Acc_ICM42688();
    Get_Gyro_ICM42688();

    Pitch_acc = -icm42688_acc_y;
    Roll_acc  =  icm42688_acc_x;
    Yaw_acc   =  icm42688_acc_z;

    Pitch_acc_F = biquad(&Pitch_acc_biquad,Pitch_acc);
    Roll_acc_F  = biquad(&Roll_acc_biquad,Roll_acc);
    Yaw_acc_F   = biquad(&Yaw_acc_biquad,Yaw_acc);

    Pitch_g = -icm42688_gyro_y;
    Roll_g  =  icm42688_gyro_x;
    Yaw_g   =  icm42688_gyro_z;

    Pitch_g_F = biquad(&Pitch_g_biquad,Pitch_g);
    Roll_g_F  = biquad(&Roll_g_biquad,Roll_g);
    Yaw_g_F   = biquad(&Yaw_g_biquad,Yaw_g);

    MahonyAHRSupdateIMU(icm42688_gyro_x,icm42688_gyro_y,icm42688_gyro_z,icm42688_acc_x,icm42688_acc_y,icm42688_acc_z);
    Pitch_a  = Rad2Ang * Pitch_a_Pi;
    Roll_a   = Rad2Ang * Roll_a_Pi;
    Yaw_a    = Rad2Ang * Yaw_a_Pi;

    Yaw_a_Pi_genrl = Yaw_a_Pi-Yaw_A0;

    if(IRQCount%4==0){ /** 1ms ******************************/
        SPI_switch_to_PAW3395();
        PAW3395_READ();
        X_acc=1E-5*(Pitch_acc_F+1000.0f*sinf(Roll_a_Pi))/* - PAW3395_M_K1*(Roll_g_F-Roll_g_F_last) * cosf(Roll_a_Pi)*/;
        Y_acc=1E-5*(Roll_acc_F-1000.0f*sinf(Pitch_a_Pi))/* + PAW3395_M_K1*(Pitch_g_F-Pitch_g_F_last) * cosf(Pitch_a_Pi)*/;
        if(PAW3395_ERR||(aim_y_v!=0 && PAW3395_Y==0)){
            X_V=X_V_last+X_acc;
            Y_V=Y_V_last+Y_acc;
        }else{
            X_V =  PAW3395_X*PAW3395_V_K + PAW3395_M_K*Roll_g_F * cosf(Roll_a_Pi);
            Y_V = -PAW3395_Y*PAW3395_V_K - PAW3395_M_K*Pitch_g_F * cosf(Pitch_a_Pi);
        }
        if((X_V-X_V_last)>PAW3395_MAX_dV){
            X_V=X_V_last+PAW3395_MAX_dV;
        }else if((X_V-X_V_last)<-PAW3395_MAX_dV){
            X_V=X_V_last-PAW3395_MAX_dV;
        }
        if((Y_V-Y_V_last)>PAW3395_MAX_dV){
            Y_V=Y_V_last+PAW3395_MAX_dV;
        }else if((Y_V-Y_V_last)<-PAW3395_MAX_dV){
            Y_V=Y_V_last-PAW3395_MAX_dV;
        }
        X_V_F = biquad(&X_V_biquad,X_V);
        Y_V_F = biquad(&Y_V_biquad,Y_V);
        X_V_last=X_V;
        Y_V_last=Y_V;
        X_Pos+=X_V_F*0.001f;
        Total_mileage+=Y_V_F*0.001f;
        Yaw_a_Pi_local = Yaw_a_Pi-Yaw_A0_local*Ang2Rad;
        IN_NAV_local_dY+=0.001f*(Y_V_F*sinf(Yaw_a_Pi_local)-X_V_F*cosf(Yaw_a_Pi_local));
        PAW3395_X_SUM+=PAW3395_X;
        PAW3395_Y_SUM+=PAW3395_Y;
        SPI_switch_to_ICM42688();
    }

    if(navgation_type==REMOTE){
        if(CRSF_Link.uplink_Link_quality<10){//丢控上锁
            Motor_Enable=0;
            RUNNING=0;
        }else if(CRSF_Cnl[4]<1000){//手动上锁
            Motor_Enable=0;
            RUNNING=0;
        }else if(CRSF_Cnl[7]>1000){//油门校准
            Motor_Enable=2;
            RUNNING=0;
        }else{
            Motor_Enable=RUNNING=1;
        }
    }else{//图像模式遥控停车
        if(CRSF_Link.uplink_Link_quality>20){
            if(CRSF_Cnl[4]<1000){//手动上锁
                Motor_Enable=0;
            }
        }
        if(!Motor_Enable)RUNNING=0;
    }

    if(B2B_Timeout_cnt)B2B_Timeout_cnt--;
    if(!B2B_Timeout_cnt || Pitch_a>30 || Pitch_a< -30 || Roll_a>30 || Roll_a< -30){ //倾倒保护
        if(Motor_Enable){Motor_Enable=0;Beep_set(1000);}
    }

    static float d_aim_y_v;

    if(navgation_type==REMOTE){//遥控控制
        if(CRSF_Cnl[8]>1000){
            aim_yaw_g=-180.0f*Get_CRSF_cnl_f1(4);
            aim_x_v = 2.0f*Get_CRSF_cnl_f1(1);
        }else{
            aim_yaw_g=PID_Yaw_a(0,aim_yaw_a-Yaw_a);
            PID_X_Pos();
        }
        aim_y_v = 1.0f*(Get_CRSF_cnl_f1(2)+1.0f);
    }else{//图像控制
        aim_yaw_g=PID_Yaw_a(0,aim_yaw_a-Yaw_a);
        PID_X_Pos();
        if(RUNNING){
           if(Aim_Speed==0){//停车自动关电机
               if(Motor_Disable_cnt<8000)Motor_Disable_cnt++;
               else{
                   Motor_Enable=RUNNING=0;
               }
           }else{
               Motor_Disable_cnt=0;
           }
           aim_y_v=Aim_Speed;
        }
        else aim_y_v=0;
    }

    PID_Yaw_gyro();
    PID_X_V();
    PID_Y_V();
    if(PAW3395_ERR){
//        Duty_FB=0;
//        Duty_LR=0;
        Beep_set(10);
    }else{

    }

    Force_0_FL=Force_1_FR=Force_2_BR=Force_3_BL=0;
    Force_FB*=Force_K;
    Force_LR*=Force_K;
    Force_Yaw*=0.5f;
    if(Force_FB>0){//前进
        Force_0_FL+=Force_FB;
        Force_1_FR+=Force_FB;
    }else{
        Force_2_BR-=Force_FB;
        Force_3_BL-=Force_FB;
    }
    if(Force_LR>0){//右平移
        Force_0_FL+=Force_LR;
        Force_3_BL+=Force_LR;
    }else{
        Force_1_FR-=Force_LR;
        Force_2_BR-=Force_LR;
    }
    if(Force_Yaw>0){//左转
        Force_1_FR+=Force_Yaw;
        Force_3_BL+=Force_Yaw;
    }else{
        Force_0_FL-=Force_Yaw;
        Force_2_BR-=Force_Yaw;
    }

    Duty_0_FL=Motor_Force2Duty(Force_0_FL);
    Duty_1_FR=Motor_Force2Duty(Force_1_FR);
    Duty_2_BR=Motor_Force2Duty(Force_2_BR);
    Duty_3_BL=Motor_Force2Duty(Force_3_BL);
//    Duty_D_DO=Get_CRSF_cnl_u10000(10);
//    Duty_D_DO=9999;
    if(Duty_D_DO<9997 && Motor_Enable)Duty_D_DO+=3;
    if(!Motor_Enable)Duty_D_DO=0;

    if(Motor_Enable==1){
        MotroCtrl_PWM(Duty_0_FL,Duty_1_FR,Duty_2_BR,Duty_3_BL,Duty_D_DO);
        if(Motor_Enable_cnt<8000)Motor_Enable_cnt++;
        else RUNNING=1;
    }else{
        if(Motor_Enable==2){
            Duty_Base=Get_CRSF_cnl_u10000(3);
            MotroCtrl_PWM(Duty_Base,Duty_Base,Duty_Base,Duty_Base,Duty_Base);
        }else{
            MotroCtrl_PWM(0,0,0,0,0);
        }
        RUNNING=0;
        Motor_Enable_cnt=0;
    }

    aim_y_v_last=aim_y_v;

    if(IRQCount%216==0){ /** 54ms *****************************/
        gpio_toggle_level(B10);
        IRQCount=0;
    }
}

void Reset_Control(void){
    X_V=Y_V=0;
    X_V_last=Y_V_last=0;
    X_V_R_last=Y_V_R_last=0;
    XVIntegral=0;
    XPIntegral=0;
    YVIntegral=0;
    YGIntegral=0;
    YAIntegral[0]=0;
    YAIntegral[1]=0;
    Aim_Speed=0;
    PAW3395_X_SUM=0;
    PAW3395_Y_SUM=0;
    Total_mileage=0;
}
/* X_Line 左右***************************/
float XVIntegral;
void PID_X_V(void){
    static float XVError, XVError_last;
    aim_x_v=fclip(aim_x_v,-2.0f,2.0f);
    XVError = fclip(aim_x_v-X_V_F,-2.0f,2.0f);
    XVIntegral = fclip(XVIntegral+PIDK_AA.Ki*XVError, -200, 200);
    Force_LR = fclip(-PIDK_AC.Kp * TOTAL_MASS_gram_div_G * Y_V_F * Ang2Rad * Yaw_g_F+ PIDK_AA.Kp * XVError + XVIntegral + PIDK_AA.Kd * (XVError-XVError_last),-2000,2000);
    XVError_last = XVError;
}
float XPIntegral;
void PID_X_Pos(void){
    static float XPError, XPError_last;
    Aim_X_Pos=fclip(Aim_X_Pos,-1.0f,1.0f);
    XPError = fclip(Aim_X_Pos-X_Pos,-2.0f,2.0f);
    XPIntegral = fclip(XPIntegral+PIDK_AB.Ki*XPError, -0.2, 0.2);
    aim_x_v = fclip(PIDK_AB.Kp * XPError + XPIntegral + PIDK_AB.Kd * (XPError-XPError_last),-1,1);
    XPError_last = XPError;
}

/* Y_Line 前后***************************/
float YVIntegral;
void PID_Y_V(void){
    static float YVError, YVError_last;
    aim_y_v=fclip(aim_y_v,-3.0f,3.0f);
    YVError = fclip(aim_y_v-Y_V_F,-2.0f,2.0f);
    YVIntegral = fclip(YVIntegral+PIDK_BA.Ki*YVError, -200, 200);
    Force_FB = fclip(PIDK_BC.Kp * aim_y_v + PIDK_BA.Kp * YVError + YVIntegral + PIDK_BA.Kd * (YVError-YVError_last),-1500,1500);
    YVError_last = YVError;
}

/* Yaw ***************************/
float YGIntegral;
void PID_Yaw_gyro(void){
    static float YGError, YGError_last;
    aim_yaw_g=fclip(aim_yaw_g,-180.0f,180.0f);
    YGError = fclip(aim_yaw_g-Yaw_g_F,-180.0f,180.0f);
    YGIntegral = fclip(YGIntegral+PIDK_YG.Ki*YGError, -200, 200);
    Force_Yaw = fclip(PIDK_YV.Kp * aim_yaw_g + PIDK_YG.Kp * YGError + YGIntegral + PIDK_YG.Kd * (YGError-YGError_last),-1500,1500);
    YGError_last = YGError;
}

float YAError[PID_YA_cnl], YAIntegral[PID_YA_cnl], Pitch_A1, dYAError_MAX=0.1f;
float PID_Yaw_a(uint8 cnl, float YAError_input){
    static float dYAError[2], YAError_last[2],output_yaw_g, aim_yaw_g_MAX=180, aim_yaw_g_last[2], d_aim_yaw_g[2];
//    static uint8 regist_flag=1;
//    if(regist_flag){
//        adjust_register(adjust_type_f,0,8,0.1,&dYAError_MAX,"dYAE_MAX");
//        regist_flag=0;
//    }
//    while(aim_yaw_a >  180) aim_yaw_a-=360;
//    while(aim_yaw_a < -180) aim_yaw_a+=360;
//    YAError = Yaw_a - aim_yaw_a;
//    if(YAError>180) YAError-=360;
//    else if(YAError < -180) YAError+=360;
//    if(fabsf(YAError)<90.0f)
//        Pitch_A1 = -Roll_a * YAError / 90.0f;
//    else if(YAError>0)
//        Pitch_A1 = Roll_a * (YAError-180.0f) / 90.0f;
//    else
//        Pitch_A1 = Roll_a * (180.0f+YAError) / 90.0f;
    if(YAError_input>180) YAError_input-=360;
    else if(YAError_input < -180) YAError_input+=360;
    YAError[cnl] = fclip(YAError_input,-30,30);
    dYAError[cnl] = YAError[cnl] - YAError_last[cnl];
//    if(Now_Speed_abs>1500.0f){
//        dYAError_MAX = 0.05f;
//    }else if(Now_Speed_abs>1200.0f){
//        dYAError_MAX = 0.2f - 0.15f*(1500.0f-Now_Speed_abs)/300.0f;
//    }else if(Now_Speed_abs>1000.0f){
//        dYAError_MAX = 0.3f - 0.1f*(1200.0f-Now_Speed_abs)/200.0f;
//    }else{
//        dYAError_MAX = 0.3f;
//    }
    if(dYAError[cnl]>dYAError_MAX){
        YAError[cnl]=YAError_last[cnl]+dYAError_MAX;
    }else if(dYAError[cnl]<-dYAError_MAX){
        YAError[cnl]=YAError_last[cnl]-dYAError_MAX;
    }
    YAIntegral[cnl] = fclip(YAIntegral[cnl] + PIDK_YA.Ki*YAError[cnl], -30, 30);
//    aim_yaw_g_MAX = 1.5f*Now_Speed_abs;
    output_yaw_g = fclip(PIDK_YA.Kp * (YAError[cnl]) + YAIntegral[cnl] + PIDK_YA.Kd * (YAError[cnl]-YAError_last[cnl]),-aim_yaw_g_MAX,aim_yaw_g_MAX);

//    d_aim_yaw_g[cnl]=output_yaw_g-aim_yaw_g_last[cnl];
//    if(d_aim_yaw_g[cnl]>20){
//        output_yaw_g=aim_yaw_g_last[cnl]+100;
//    }else if(d_aim_yaw_g[cnl]<-20){
//        output_yaw_g=aim_yaw_g_last[cnl]-100;
//    }
//    aim_yaw_g_last[cnl] = aim_yaw_g;

//    YAError_LL = YAError_last;
    YAError_last[cnl] = YAError[cnl];
    return output_yaw_g;
}

void biquad_filter_init(biquad_state *state, biquad_type type, int fs, float fc, float q_value)
{
    sample_t w0, sin_w0, cos_w0, alpha;
    sample_t b0, b1, b2, a0, a1, a2;

    w0 = 2 * PI * fc / fs;
    sin_w0 = sinf(w0);
    cos_w0 = cosf(w0);
    alpha = sin_w0 / (2.0 * q_value);

    switch(type)
    {
    case BIQUAD_LOWPASS:
        b0 = (1.0 - cos_w0) / 2.0;
        b1 = b0 * 2;
        b2 = b0;
        a0 = 1.0 + alpha;
        a1 = -2.0 * cos_w0;
        a2 = 1.0 - alpha;
        break;
    case BIQUAD_HIGHPASS:
        b0 = (1.0 + cos_w0) / 2.0;
        b1 = -b0 * 2;
        b2 = b0;
        a0 = 1.0 + alpha;
        a1 = -2.0 * cos_w0;
        a2 = 1.0 - alpha;
        break;
    case BIQUAD_BANDPASS_PEAK:
        b0 = alpha;
        b1 = 0.0;
        b2 = -alpha;
        a0 = 1.0 + alpha;
        a1 = -2.0 * cos_w0;
        a2 = 1.0 - alpha;
        break;
    case BIQUAD_BANDSTOP_NOTCH:
        b0 = 1.0;
        b1 = -2.0 * cos_w0;
        b2 = 1.0;
        a0 = 1.0 + alpha;
        a1 = -2.0 * cos_w0;
        a2 = 1.0 - alpha;
        break;
    }
    state->a0 = b0 / a0;
    state->a1 = b1 / a0;
    state->a2 = b2 / a0;
    state->a3 = a1 / a0;
    state->a4 = a2 / a0;
    state->x1 = state->x2 = 0.0;
    state->y1 = state->y2 = 0.0;
}

//biquard滤波
sample_t biquad(biquad_state *state, sample_t data)
{
    sample_t result = 0;
    result = state->a0 * data + state->a1 * state->x1 + state->a2 * state->x2 -  state->a3 * state->y1 - state->a4 * state->y2;
    state->x2 = state->x1;
    state->x1 = data;
    state->y2 = state->y1;
    state->y1 = result;
    return result;

}

uint16 Motor_Force2Duty(float Force){
    Force=fclip(Force+29.0f,0.0f,2000.0f);
    return clip_u16((5.2263699693729E-07)*Force*Force*Force-(2.513823548460756E-03)*Force*Force+6.9674068008572*Force+1299.7906516429536,1500,7500);
}
