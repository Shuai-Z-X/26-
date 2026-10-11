
#include "stm32f10x.h"
#include <stdio.h>
#include <math.h>

// ===================== 用户配置区 =====================
#define ENCODER_PPR         13          // MG545 编码器线数
#define REDUCTION_RATIO     1.0f        // 无减速器
#define SAMPLE_TIME_MS      10
#define SAMPLE_TIME_S       0.01f
#define TARGET_SPEED        100.0f      // 目标转速 rad/s

// PID 参数
#define PID_KP              20.0f
#define PID_KI              1.0f
#define PID_KD              0.01f

// 卡尔曼滤波参数
#define KALMAN_Q            0.3f
#define KALMAN_R            8.0f

// PWM 满幅值（TIM3 周期 7200，10kHz）
#define PWM_MAX             7200
#define PWM_MIN             -7200

// ===================== 全局变量 =====================
volatile float g_TargetSpeed = TARGET_SPEED;
volatile float g_ActualSpeed = 0.0f;
volatile float g_RawSpeed = 0.0f;
volatile float g_PWMOutput = 0.0f;
volatile int32_t g_LastEncoder = 0;

typedef struct {
    float Kp, Ki, Kd;
    float Error, LastError;
    float Integral;
    float Output;
} PID_TypeDef;

typedef struct {
    float x, P, Q, R, K;
} Kalman1D;

PID_TypeDef g_PID;
Kalman1D g_KF;

// ===================== 函数声明 =====================
void RCC_Config(void);
void GPIO_Config(void);
void TIM3_PWM_Config(void);
void TIM2_Encoder_Config(void);
void TIM4_Sample_Config(void);
void USART1_Config(void);
void PID_Init(void);
float PID_Calc(float target, float actual);
void Motor_SetPWM(int16_t pwm);
void Kalman_Init(Kalman1D *k, float Q, float R);
float Kalman_Update(Kalman1D *k, float measurement, float control);
int16_t SoftStart(int16_t target, int16_t current, int16_t max_step);
int fputc(int ch, FILE *f);
void VOFA_SendFrame(const float *data, uint8_t channel_count);

// ===================== 软启动 =====================
int16_t SoftStart(int16_t target, int16_t current, int16_t max_step) {
    if(target > current) {
        if(target - current > max_step) return current + max_step;
    } else if(target < current) {
        if(current - target > max_step) return current - max_step;
    }
    return target;
}

// ===================== 卡尔曼 =====================
void Kalman_Init(Kalman1D *k, float Q, float R) {
    k->x = 0.0f; k->P = 10.0f; k->Q = Q; k->R = R; k->K = 0.0f;
}

float Kalman_Update(Kalman1D *k, float measurement, float control) {
    float B = 6.0f;
    k->x = k->x + B * control;
    k->P = k->P + k->Q;
    k->K = k->P / (k->P + k->R);
    k->x = k->x + k->K * (measurement - k->x);
    k->P = (1.0f - k->K) * k->P;
    return k->x;
}

// ===================== PID =====================
void PID_Init(void) {
    g_PID.Kp = PID_KP; g_PID.Ki = PID_KI; g_PID.Kd = PID_KD;
    g_PID.Error = g_PID.LastError = 0;
    g_PID.Integral = 0; g_PID.Output = 0;
}

float PID_Calc(float target, float actual) {
    float derivative;

    g_PID.Error = target - actual;
    
    derivative = (g_PID.Error - g_PID.LastError) / SAMPLE_TIME_S;
    g_PID.Output = g_PID.Kp * g_PID.Error 
                 + g_PID.Ki * g_PID.Integral 
                 + g_PID.Kd * derivative;
    g_PID.LastError = g_PID.Error;
    
    if(g_PID.Output > PWM_MAX)  g_PID.Output = PWM_MAX;
    if(g_PID.Output < PWM_MIN)  g_PID.Output = PWM_MIN;

    /* 积分抗饱和：输出已经顶到上限/下限时不再继续积分 */
    if(!((g_PID.Output >= PWM_MAX && g_PID.Error > 0) ||
         (g_PID.Output <= PWM_MIN && g_PID.Error < 0))) {
        g_PID.Integral += g_PID.Error * SAMPLE_TIME_S;
    }
    if(g_PID.Integral > 2000)  g_PID.Integral = 2000;
    if(g_PID.Integral < -2000) g_PID.Integral = -2000;
    
    return g_PID.Output;
}

// ===================== VOFA+ JustFloat =====================
// 帧格式：N 个 float（小端 4 字节）+ 帧尾 0x00 0x00 0x80 0x7F
void VOFA_SendFrame(const float *data, uint8_t channel_count) {
    uint8_t i, j;
    const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};
    
    for(i = 0; i < channel_count; i++) {
        const uint8_t *p = (const uint8_t *)&data[i];
        for(j = 0; j < 4; j++) {
            while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
            USART_SendData(USART1, p[j]);
        }
    }
    
    for(i = 0; i < 4; i++) {
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, tail[i]);
    }
}

// ===================== 电机控制 =====================
void Motor_SetPWM(int16_t pwm) {
    if(pwm > 0) {
        TIM_SetCompare1(TIM3, pwm);
        TIM_SetCompare2(TIM3, 0);
    } else if(pwm < 0) {
        TIM_SetCompare1(TIM3, 0);
        TIM_SetCompare2(TIM3, -pwm);
    } else {
        TIM_SetCompare1(TIM3, 0);
        TIM_SetCompare2(TIM3, 0);
    }
}

// ===================== 主函数 =====================
int main(void) {
    RCC_Config();
    GPIO_Config();
    TIM3_PWM_Config();
    TIM2_Encoder_Config();
    TIM4_Sample_Config();
    USART1_Config();
    PID_Init();
    Kalman_Init(&g_KF, KALMAN_Q, KALMAN_R);
    
    // 使能 A4950（PB0 拉高）
    GPIO_SetBits(GPIOB, GPIO_Pin_0);
    while(1);
}

// ===================== 定时中断 =====================
void TIM4_IRQHandler(void) {
    if(TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
        
        // 1. 读编码器（TIM2 是 4 倍频计数，16 位计数器）
        int32_t current = (int32_t)TIM_GetCounter(TIM2);
        int32_t delta = current - g_LastEncoder;
        g_LastEncoder = current;
        
        // 16位溢出修正
        if(delta > 32767)  delta -= 65536;
        if(delta < -32768) delta += 65536;
        
        // 2. 算速度 rad/s
        float ppr_real = ENCODER_PPR * REDUCTION_RATIO;
        g_RawSpeed = (float)delta * 2.0f * 3.1415926f 
                    / (4.0f * ppr_real * SAMPLE_TIME_S);
        
        // 3. 卡尔曼滤波
        float control = g_PWMOutput / PWM_MAX;
        if(control > 1.0f) control = 1.0f;
        if(control < -1.0f) control = -1.0f;
        g_ActualSpeed = Kalman_Update(&g_KF, g_RawSpeed, control);
        
        // 4. PID 计算
        int16_t target_pwm = (int16_t)PID_Calc(g_TargetSpeed, g_ActualSpeed);
        
        // 5. 软启动限幅
        static int16_t last_pwm = 0;
        int16_t soft_pwm = SoftStart(target_pwm, last_pwm, 50);
        last_pwm = soft_pwm;
        
        // 6. 输出到电机
        Motor_SetPWM(soft_pwm);
        g_PWMOutput = soft_pwm;
        
        // 7. 每 10ms 发一帧给 VOFA+（JustFloat）
        //    通道 1：PA6(CH1) 占空比%；通道 2：PA7(CH2) 占空比%
        //    通道 3：滤波后转速 rad/s；通道 4：目标转速 rad/s
        float vofa_frame[4];
        vofa_frame[0] = (g_PWMOutput > 0) ?
                        (g_PWMOutput / PWM_MAX * 100.0f) : 0.0f;
        vofa_frame[1] = (g_PWMOutput < 0) ?
                        (-g_PWMOutput / PWM_MAX * 100.0f) : 0.0f;
        vofa_frame[2] = g_ActualSpeed;
        vofa_frame[3] = g_TargetSpeed;
        VOFA_SendFrame(vofa_frame, 4);
    }
}

// ===================== 外设配置 =====================
void RCC_Config(void) {
    RCC_APB2PeriphClockCmd(
        RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | 
        RCC_APB2Periph_AFIO | RCC_APB2Periph_USART1, 
        ENABLE
    );
    RCC_APB1PeriphClockCmd(
        RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, 
        ENABLE
    );
}

void GPIO_Config(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    
    // PA6, PA7 -> TIM3 CH1, CH2 (PWM 输出)
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // PB0 -> A4950 EN (使能)
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    // PA0, PA1 -> TIM2 CH1, CH2 (编码器输入，浮空)
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // PA9 -> USART1_TX
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void TIM3_PWM_Config(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    TIM_OCInitTypeDef TIM_OCInitStruct;
    
    // 72MHz / 7200 = 10kHz
    TIM_TimeBaseStruct.TIM_Period = PWM_MAX - 1;
    TIM_TimeBaseStruct.TIM_Prescaler = 0;
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStruct);
    
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_Pulse = 0;
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
    
    TIM_OC1Init(TIM3, &TIM_OCInitStruct);   // CH1 -> PA6
    TIM_OC2Init(TIM3, &TIM_OCInitStruct);   // CH2 -> PA7
    
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
}

void TIM2_Encoder_Config(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    
    TIM_TimeBaseStruct.TIM_Period = 65535;      // 16位最大
    TIM_TimeBaseStruct.TIM_Prescaler = 0;
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStruct);
    
    // 编码器模式 TI1+TI2：A/B 相都计数，4 倍频
    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising,
                               TIM_ICPolarity_Rising);
    TIM_SetCounter(TIM2, 0);
    TIM_Cmd(TIM2, ENABLE);
}

void TIM4_Sample_Config(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
    
    // 72MHz / 7200 / 100 = 100Hz -> 10ms
    TIM_TimeBaseStruct.TIM_Period = 100 - 1;
    TIM_TimeBaseStruct.TIM_Prescaler = 7200 - 1;
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStruct);
    
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    
    NVIC_InitStruct.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
    
    TIM_Cmd(TIM4, ENABLE);
}

void USART1_Config(void) {
    USART_InitTypeDef USART_InitStruct;
    
    USART_InitStruct.USART_BaudRate = 115200;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
     USART_InitStruct.USART_Mode = USART_Mode_Tx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &USART_InitStruct);
    USART_Cmd(USART1, ENABLE);
}

// ===================== 串口重定向 =====================
int fputc(int ch, FILE *f) {
    USART_SendData(USART1, (uint8_t)ch);
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    return ch;
}
// 放在 main.c 的最后，所有函数外面
