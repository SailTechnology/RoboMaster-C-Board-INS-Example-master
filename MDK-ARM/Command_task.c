/*---------------------------------------------------------------------------
 *  ����������ģʽ ���� ���� PID ����
 *  �⻷ : ��̬�� PID  -> �����������ٶ� (deg/s)
 *  �ڻ� : ���ٶ� PID  -> ������ػ����, ������� PWM
 *  ���� : |Roll| �� |Pitch| > 80�� ʱ�Զ�����������������ٽ���
 *---------------------------------------------------------------------------*/
#include "command_task.h"
#include "ins_task.h"        /*  INS.Roll INS.Pitch INS.Yaw  INS.Gyro[X..Z] */
#include "Receiver_task.h"   /*  extern int RC_Throttle, RC_Roll, ...       */
#include "bsp_PWM.h"         /*  TIM_Set_PWM(&htim1, CH, val)               */
#include <math.h>

/*========================= ���� & �� ======================================*/
#define MAX_TILT_ANGLE   30.0f
#define MAX_RATE_CMD     250.0f
#define MAX_YAW_RATE     180.0f

#define PWM_MIN          1000
#define PWM_MAX          2000

#define RC_DEADZONE      5
#define INTEGRATOR_LIMIT 400.0f

#define constrain(x,l,h) ((x)<(l)?(l):((x)>(h)?(h):(x)))

/*========================= PID �ṹ�� =====================================*/
typedef struct {
    float kp, ki, kd;
    float integrator;
    float last_err;
} PID_t;

/*-------------------- �Ķ� PID ʵ�� ---------------------------------------*/
static PID_t ang_pid_roll , ang_pid_pitch , ang_pid_yaw;   /* �⻷  */
static PID_t rate_pid_roll, rate_pid_pitch, rate_pid_yaw;  /* �ڻ�  */

/*========================= �û������� =====================================*/


/* �⻷ *///0.15f  0.001f
//#define ANG_KP_RP   0.15f
//#define ANG_KI_RP   0.001f
//#define ANG_KP_YAW  0.02f
/* �ڻ�14.0f 0.0f 1.0f*/
//#define RATE_KP_RP  14.0f
//#define RATE_KI_RP  0.0f
//#define RATE_KD_RP  1.0f
//#define RATE_KP_YAW 20.0f
//#define RATE_KI_YAW 0.05f

/* �⻷ *///0.15f  0.001f
#define ANG_KP_RP   0.15f
#define ANG_KI_RP   0.001f
#define ANG_KP_YAW  0.04f
/* �ڻ�14.0f 0.0f 1.0f*/
#define RATE_KP_RP  14.0f
#define RATE_KI_RP  0.0f
#define RATE_KD_RP  1.0f
#define RATE_KP_YAW 40.0f
#define RATE_KI_YAW 0.05f
/* �����޷� */
#define THR_MAX_NORM 0.85f
/* ������ֵ */
#define AUTOLOCK_ANGLE 75.0f

/*========================= ȫ��״̬���� ===================================*/
static float des_roll=0, des_pitch=0, des_yaw=0;
static float des_rate_roll=0, des_rate_pitch=0, des_rate_yaw=0;

/* �Զ�������־ */
static uint8_t auto_lock = 0;

/*========================= �ڲ����� =======================================*/
static void PID_Init(PID_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp; pid->ki = ki; pid->kd = kd;
    pid->integrator = 0.0f; pid->last_err = 0.0f;
}

static float PID_Update(PID_t *pid, float err, float rate)
{
    float p = pid->kp * err;
    pid->integrator += err;
    pid->integrator = constrain(pid->integrator, -INTEGRATOR_LIMIT, INTEGRATOR_LIMIT);
    float i = pid->ki * pid->integrator;
    float d = -pid->kd * rate;
    pid->last_err = err;
    return p + i + d;
}

/*========================= �ӿ�ʵ�� =======================================*/
void Command_Task_Init(void)
{
    PID_Init(&ang_pid_roll , ANG_KP_RP , ANG_KI_RP , 0);
    PID_Init(&ang_pid_pitch, ANG_KP_RP , ANG_KI_RP , 0);
    PID_Init(&ang_pid_yaw  , ANG_KP_YAW, 0         , 0);

    PID_Init(&rate_pid_roll , RATE_KP_RP , RATE_KI_RP , RATE_KD_RP);
    PID_Init(&rate_pid_pitch, RATE_KP_RP , RATE_KI_RP , RATE_KD_RP);
    PID_Init(&rate_pid_yaw  , RATE_KP_YAW, RATE_KI_YAW, 0);

    des_roll  = INS.Roll;
    des_pitch = INS.Pitch;
    des_yaw   = INS.Yaw;

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    TIM_Set_PWM(&htim1, TIM_CHANNEL_1, PWM_MIN);
    TIM_Set_PWM(&htim1, TIM_CHANNEL_2, PWM_MIN);
    TIM_Set_PWM(&htim1, TIM_CHANNEL_3, PWM_MIN);
    TIM_Set_PWM(&htim1, TIM_CHANNEL_4, PWM_MIN);
}

void Command_Task_Loop(void)
{
    /* ====== 0. ���� RC Arm ���Զ����� ====== */
    uint8_t arm_ok = RC_Arm;          /* ��ѭ���Ƿ����������� */

    /* ���Ѵ����Զ�������������û���ۣ��������� */
    if (auto_lock) arm_ok = 0;

    /* ��� = ����Զ��� */
    if (!RC_Arm) auto_lock = 0;

    /* ====== ���� (>80��) �Զ�������� ====== */
    if (arm_ok && (fabsf(INS.Roll) > AUTOLOCK_ANGLE || fabsf(INS.Pitch) > AUTOLOCK_ANGLE)) {
        auto_lock = 1;
        arm_ok = 0;                    /* ����ͣ�� */
    }

    /* ====== 0+1. ��ȫ������ͣ�ж� ====== */
    if (RC_Lost || RC_Kill || !arm_ok) {
        TIM_Set_PWM(&htim1,TIM_CHANNEL_1,PWM_MIN);
        TIM_Set_PWM(&htim1,TIM_CHANNEL_2,PWM_MIN);
        TIM_Set_PWM(&htim1,TIM_CHANNEL_3,PWM_MIN);
        TIM_Set_PWM(&htim1,TIM_CHANNEL_4,PWM_MIN);
        ang_pid_roll.integrator = ang_pid_pitch.integrator = ang_pid_yaw.integrator = 0;
        rate_pid_roll.integrator = rate_pid_pitch.integrator = rate_pid_yaw.integrator = 0;
        des_yaw = INS.Yaw;
        return;
    }

    /*=================== 1. ��ȡң���� & ������̬�� ===========*/
    float roll_in  = (float)RC_Roll  - 1500.0f;
    float pitch_in = (float)RC_Pitch - 1500.0f;
    float yaw_in   = (float)RC_Yaw   - 1500.0f;
    pitch_in = -pitch_in;
    yaw_in = -yaw_in;

    if (fabsf(roll_in)  < RC_DEADZONE) roll_in  = 0;
    if (fabsf(pitch_in) < RC_DEADZONE) pitch_in = 0;
    if (fabsf(yaw_in)   < RC_DEADZONE) yaw_in   = 0;

    des_roll  = (roll_in  / 500.0f) *  MAX_TILT_ANGLE;
    des_pitch = -(pitch_in / 500.0f) * MAX_TILT_ANGLE;
    const float dt = 0.002f;
    des_yaw += (yaw_in / 500.0f) * MAX_YAW_RATE * dt;
    if (des_yaw >  180.0f) des_yaw -= 360.0f;
    if (des_yaw < -180.0f) des_yaw += 360.0f;

    /*=================== 2. �ǶȻ� -> �������ٶ� ==============*/
    float err_ang_roll  = des_roll  - INS.Roll;
    float err_ang_pitch = des_pitch - INS.Pitch;
    float err_ang_yaw   = des_yaw   - INS.Yaw;
    if (err_ang_yaw > 180.0f)  err_ang_yaw -= 360.0f;
    if (err_ang_yaw < -180.0f) err_ang_yaw += 360.0f;

    des_rate_roll  = PID_Update(&ang_pid_roll , err_ang_roll , 0);
    des_rate_pitch = PID_Update(&ang_pid_pitch, err_ang_pitch, 0);
    des_rate_yaw   = PID_Update(&ang_pid_yaw  , err_ang_yaw  , 0);

    des_rate_roll  = constrain(des_rate_roll ,  -MAX_RATE_CMD, MAX_RATE_CMD);
    des_rate_pitch = constrain(des_rate_pitch, -MAX_RATE_CMD, MAX_RATE_CMD);
    des_rate_yaw   = constrain(des_rate_yaw  , -MAX_RATE_CMD, MAX_RATE_CMD);

    /*=================== 3. ���ʻ����� =======================*/
    float err_rate_roll  = des_rate_roll  - INS.Gyro[X];
    float err_rate_pitch = des_rate_pitch - INS.Gyro[Y];
    float err_rate_yaw   = des_rate_yaw   - INS.Gyro[Z];

    float roll_out  = PID_Update(&rate_pid_roll , err_rate_roll ,  INS.Gyro[X]);
    float pitch_out = PID_Update(&rate_pid_pitch, err_rate_pitch,  INS.Gyro[Y]);
    float yaw_out   = PID_Update(&rate_pid_yaw  , err_rate_yaw  ,  INS.Gyro[Z]);

    /*=================== 4. ���Ż�� (X Frame) ===============*/
    float base_throttle = (float)RC_Throttle;

    float pwm1 = base_throttle + roll_out - pitch_out + yaw_out;  /* ǰ�� */
    float pwm2 = base_throttle - roll_out - pitch_out - yaw_out;  /* ǰ�� */
    float pwm3 = base_throttle - roll_out + pitch_out + yaw_out;  /* ���� */
    float pwm4 = base_throttle + roll_out + pitch_out - yaw_out;  /* ���� */

    pwm1 = constrain(pwm1, PWM_MIN, PWM_MAX);
    pwm2 = constrain(pwm2, PWM_MIN, PWM_MAX);
    pwm3 = constrain(pwm3, PWM_MIN, PWM_MAX);
    pwm4 = constrain(pwm4, PWM_MIN, PWM_MAX);

    TIM_Set_PWM(&htim1, TIM_CHANNEL_1, (uint16_t)pwm1);
    TIM_Set_PWM(&htim1, TIM_CHANNEL_2, (uint16_t)pwm2);
    TIM_Set_PWM(&htim1, TIM_CHANNEL_3, (uint16_t)pwm3);
    TIM_Set_PWM(&htim1, TIM_CHANNEL_4, (uint16_t)pwm4);
}
