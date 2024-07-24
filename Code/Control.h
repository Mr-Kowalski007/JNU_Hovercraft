#ifndef SRC_USER_CONTROL_H_
#define SRC_USER_CONTROL_H_
#include "zf_common_headfile.h"

/* 角度 */
extern float Roll_A0, Pitch_A0, Yaw_A0;
extern float Yaw_Lock_a;
extern float Pitch_a, Roll_a, Yaw_a;
extern float Pitch_a_Pi, Roll_a_Pi, Roll_error_Pi, Yaw_a_Pi, Yaw_a_Pi_genrl, Yaw_a_Pi_local;
extern float Pitch_a_last, Roll_a_last, Yaw_a_last;
extern float aim_roll_a,aim_pitch_a,aim_yaw_a,d_aim_yaw_a;
/* 角速度 */
extern float Pitch_g, Roll_g, Yaw_g;
extern float Pitch_g_F, Roll_g_F, Yaw_g_F;
extern float gyro_x_correction, gyro_y_correction, gyro_z_correction;//零漂修正
extern float Pitch_g_last, Roll_g_last, Yaw_g_last;
extern float aim_x_v,aim_y_v,aim_y_v_last,aim_yaw_g;
extern float X_V,Y_V;
extern float X_V_F,Y_V_F;
/* 加速度 */
extern float Pitch_acc, Roll_acc, Yaw_acc;
extern float Pitch_acc_F, Roll_acc_F, Yaw_acc_F;
extern float X_acc,Y_acc;

/* 位置 */
extern float Aim_X_Pos;
extern float X_Pos;
extern float Total_mileage;
/* 速度 */
extern volatile float Aim_Speed, Now_Speed,Now_Speed_abs;
extern volatile sint32 PAW3395_X_SUM,PAW3395_Y_SUM;
extern uint8 Speed_Gear;
/* 标志位 */
extern uint8 Motor_Enable,ALLOW_RUNNING;
extern uint16 Motor_Disable_cnt;
extern uint8 Yaw_Lock, Yaw_Enable, Yaw_g_maintain, lean_correction;
extern sint8 Dynamic_Speed;
extern uint8 Dynamic_PIDK, Slope_Mode, RUSHING,Slope_Use_PV_Ki,PS2_MODE;

typedef struct{
    float Kp;
    float Ki;
    float Kd;
}PID_t;
extern PID_t* PIDK_Debug_Group[4][3];

extern float
        Force_FB,
        Force_LR,
        Force_Yaw;
extern uint16
        Duty_Base,
        Duty_0_FL,
        Duty_1_FR,
        Duty_2_BR,
        Duty_3_BL,
        Duty_D_DO;

void Filter_Init(void);
void Control(void);

void Reset_Control(void);
extern float XVIntegral;
void PID_X_V(void);
extern float XPIntegral;
void PID_X_Pos(void);
extern float YVIntegral;
void PID_Y_V(void);
extern float YGIntegral;
void PID_Yaw_gyro(void);
#define PID_YA_cnl 2
extern float YAError[PID_YA_cnl], YAIntegral[PID_YA_cnl], Pitch_A1, dYAError_MAX;
float PID_Yaw_a(uint8 cnl, float YAError_input);


typedef float sample_t;

typedef enum {
    BIQUAD_LOWPASS,
    BIQUAD_HIGHPASS,
    BIQUAD_BANDPASS_PEAK,
    BIQUAD_BANDSTOP_NOTCH,
}biquad_type;

typedef struct
{
    sample_t a0, a1, a2, a3, a4;
    sample_t x1, x2, y1, y2;
}biquad_state;

void biquad_filter_init(biquad_state *state, biquad_type type, int fs, float fc, float q_value);
sample_t biquad(biquad_state *state, sample_t data);

uint16 Motor_Force2Duty(float Force);

#endif /* SRC_USER_CONTROL_H_ */
