#include "stm32f10x.h"

/* ===================== Config ===================== */
#define ENCODER_PPR         13          /* MG545 encoder lines */
#define SAMPLE_TIME_S       0.01f       /* motor control every 10ms */
#define TARGET_SPEED        100.0f      /* rad/s */

#define PWM_MAX             7200
#define PWM_MIN             -7200

/* Motor controller (feed-forward + trim) */
#define PID_KP              5.0f
#define PID_KI              2.0f
#define PID_KD              0.0f
#define FF_DUTY_PERCENT     25.0f
#define FF_STEP_PERCENT     3.0f

/* Encoder counts per revolution = PPR * 4 (x4 quadrature) */
#define ENC_CNT_PER_REV     52
#define VISIBLE_HALF_CNT    26

/* Hold one letter for N visible half-turns (about N*31ms at 100rad/s) */
#define LETTER_HOLD_HALVES  32

/* 0..51: rotate the displayed sector to the visible area by hand */
#define DISPLAY_PHASE_OFFSET 0

/* LED pins (avoid PA0/PA1 encoder and PA6/PA7 motor PWM) */
#define LED_DS_PORT         GPIOA
#define LED_DS_PIN          GPIO_Pin_2
#define LED_SHCP_PORT       GPIOA
#define LED_SHCP_PIN        GPIO_Pin_3
#define LED_STCP_PORT       GPIOA
#define LED_STCP_PIN        GPIO_Pin_4

/* 1: glyph row0 maps to HC595 bit17, row17 to bit0 */
#define LED_ROW_INVERT      1

/* ===================== Globals ===================== */
volatile uint8_t g_10msFlag = 0;
volatile float g_ActualSpeed = 0.0f;
volatile float g_RawSpeed = 0.0f;
volatile float g_PWMOutput = 0.0f;
volatile float g_TargetSpeed = TARGET_SPEED;

static int32_t g_encPos = 0;
static float g_integral = 0.0f;
static float g_lastError = 0.0f;
static float g_ffDuty = 0.0f;

/* ===================== BISTU 18x18 glyphs ===================== */
static const char glyph_B[18][19] = {
    "...###########....",
    "...#.........#....",
    "...#.........#....",
    "...#.........#....",
    "...#.........#....",
    "...#.........#....",
    "...#.........#....",
    "...#.........#....",
    "...###########....",
    "...#.........#....",
    "...#.........#....",
    "...#.........#....",
    "...#.........#....",
    "...#.........#....",
    "...#.........#....",
    "...#.........#....",
    "...#.........#....",
    "...###########....",
};

static const char glyph_I[18][19] = {
    "....##########....",
    "........##........",
    "........##........",
    "........##........",
    "........##........",
    "........##........",
    "........##........",
    "........##........",
    "........##........",
    "........##........",
    "........##........",
    "........##........",
    "........##........",
    "........##........",
    "........##........",
    "........##........",
    "........##........",
    "....##########....",
};

static const char glyph_S[18][19] = {
    "....##########....",
    "....#.............",
    "....#.............",
    "....#.............",
    "....#.............",
    "....#.............",
    "....#.............",
    "....#.............",
    "....##########....",
    ".............#....",
    ".............#....",
    ".............#....",
    ".............#....",
    ".............#....",
    ".............#....",
    ".............#....",
    ".............#....",
    "....##########....",
};

static const char glyph_T[18][19] = {
    "...############...",
    "........##........",
    "........##........",
    "........##........",
    "........##........",
    "........##........",
    "........##........",
    "........##........",
    "........##........",
    "........##........",
    "........##........",
    "........##........",
    "........##........",
    "........##........",
    "........##........",
    "........##........",
    "........##........",
    "........##........",
};

static const char glyph_U[18][19] = {
    "....#........#....",
    "....#........#....",
    "....#........#....",
    "....#........#....",
    "....#........#....",
    "....#........#....",
    "....#........#....",
    "....#........#....",
    "....#........#....",
    "....#........#....",
    "....#........#....",
    "....#........#....",
    "....#........#....",
    "....#........#....",
    "....#........#....",
    "....#........#....",
    "....#........#....",
    "....##########....",
};

typedef const char (*GlyphPtr)[19];

static const GlyphPtr g_glyphs[5] = {
    glyph_B, glyph_I, glyph_S, glyph_T, glyph_U
};

/* Precomputed column masks: [letter][column0..17] */
static uint32_t g_columns[5][18];

/* ===================== HC595 driver ===================== */
static void led_delay_us(uint32_t n)
{
    volatile uint32_t i;
    for(i = 0; i < n * 12; i++);
}

static void HC595_SHCP_Pulse(void)
{
    GPIO_SetBits(LED_SHCP_PORT, LED_SHCP_PIN);
    led_delay_us(1);
    GPIO_ResetBits(LED_SHCP_PORT, LED_SHCP_PIN);
    led_delay_us(1);
}

static void HC595_STCP_Pulse(void)
{
    GPIO_SetBits(LED_STCP_PORT, LED_STCP_PIN);
    led_delay_us(1);
    GPIO_ResetBits(LED_STCP_PORT, LED_STCP_PIN);
    led_delay_us(1);
}

static void HC595_SendByte(uint8_t data)
{
    uint8_t i;
    for(i = 0; i < 8; i++) {
        if(data & 0x80) GPIO_SetBits(LED_DS_PORT, LED_DS_PIN);
        else            GPIO_ResetBits(LED_DS_PORT, LED_DS_PIN);
        HC595_SHCP_Pulse();
        data <<= 1;
    }
}

/* Send last chip first: U5(bit16..23), U4(bit8..15), U3(bit0..7) */
static void HC595_Write(uint32_t mask)
{
    uint8_t u3 = (uint8_t)(mask & 0xFF);
    uint8_t u4 = (uint8_t)((mask >> 8) & 0xFF);
    uint8_t u5 = (uint8_t)((mask >> 16) & 0xFF);

    HC595_SendByte(u5);
    HC595_SendByte(u4);
    HC595_SendByte(u3);
    HC595_STCP_Pulse();
}

static void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.GPIO_Pin = LED_DS_PIN | LED_SHCP_PIN | LED_STCP_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LED_DS_PORT, &GPIO_InitStruct);

    GPIO_ResetBits(LED_DS_PORT, LED_DS_PIN);
    GPIO_ResetBits(LED_SHCP_PORT, LED_SHCP_PIN);
    GPIO_ResetBits(LED_STCP_PORT, LED_STCP_PIN);
    HC595_Write(0);
}

static void BuildColumns(void)
{
    uint8_t letter, row, col;

    for(letter = 0; letter < 5; letter++) {
        for(col = 0; col < 18; col++) {
            uint32_t mask = 0;
            for(row = 0; row < 18; row++) {
                if(g_glyphs[letter][row][col] == '#') {
#if LED_ROW_INVERT
                    mask |= (1UL << (17 - row));
#else
                    mask |= (1UL << row);
#endif
                }
            }
            g_columns[letter][col] = mask;
        }
    }
}

/* ===================== Motor PWM ===================== */
static void Motor_SetPWM(int16_t pwm)
{
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

static int16_t SoftStart(int16_t target, int16_t current, int16_t max_step)
{
    if(target > current) {
        if(target - current > max_step) return (int16_t)(current + max_step);
    } else if(target < current) {
        if(current - target > max_step) return (int16_t)(current - max_step);
    }
    return target;
}

/* ===================== Motor control (10ms) ===================== */
static void Motor_Control(void)
{
    static int32_t ctrl_last_raw = 0;
    static int16_t last_pwm = 0;
    int32_t raw, delta;
    float error, trim, base_pwm;
    int32_t target_pwm;

    raw = (int32_t)TIM_GetCounter(TIM2);
    delta = raw - ctrl_last_raw;
    ctrl_last_raw = raw;
    if(delta > 32767)  delta -= 65536;
    if(delta < -32768) delta += 65536;

    /* speed rad/s */
    g_RawSpeed = (float)delta * 2.0f * 3.1415926f
                 / (4.0f * ENCODER_PPR * SAMPLE_TIME_S);

    /* light low-pass filter instead of Kalman */
    g_ActualSpeed += (g_RawSpeed - g_ActualSpeed) * 0.25f;

    error = g_TargetSpeed - g_ActualSpeed;

    /* feed-forward ramp */
    if(g_ffDuty < FF_DUTY_PERCENT) {
        g_ffDuty += FF_STEP_PERCENT;
        if(g_ffDuty > FF_DUTY_PERCENT) g_ffDuty = FF_DUTY_PERCENT;
    }

    /* PID trim */
    trim = PID_KP * error + PID_KI * g_integral
         + PID_KD * ((error - g_lastError) / SAMPLE_TIME_S);
    g_lastError = error;

    base_pwm = g_ffDuty / 100.0f * PWM_MAX;
    target_pwm = (int32_t)(base_pwm + trim);
    if(target_pwm > PWM_MAX)  target_pwm = PWM_MAX;
    if(target_pwm < PWM_MIN)  target_pwm = PWM_MIN;

    /* anti-windup */
    if(!((target_pwm >= PWM_MAX && error > 0.0f) ||
         (target_pwm <= PWM_MIN && error < 0.0f))) {
        g_integral += error * SAMPLE_TIME_S;
    }
    if(g_integral > 2000.0f)  g_integral = 2000.0f;
    if(g_integral < -2000.0f) g_integral = -2000.0f;

    last_pwm = SoftStart((int16_t)target_pwm, last_pwm, 200);
    Motor_SetPWM(last_pwm);
    g_PWMOutput = last_pwm;
}

/* ===================== Display ===================== */
static void UpdateEncoderPosition(void)
{
    static int32_t last_raw = 0;
    int32_t raw = (int32_t)TIM_GetCounter(TIM2);
    int32_t delta = raw - last_raw;

    last_raw = raw;
    if(delta > 32767)  delta -= 65536;
    if(delta < -32768) delta += 65536;
    g_encPos += delta;
}

static void Display_Update(void)
{
    static uint8_t letter_index = 0;
    static int32_t phase_prev = 0;
    static int32_t half_count = 0;
    static int8_t last_col = -1;
    int32_t phase, col;

    /* only refresh letters when the rotor is nearly up to speed */
    if(g_ActualSpeed < 40.0f) {
        if(last_col != -1) {
            HC595_Write(0);
            last_col = -1;
        }
        return;
    }

    phase = ((g_encPos - DISPLAY_PHASE_OFFSET) % ENC_CNT_PER_REV + ENC_CNT_PER_REV)
            % ENC_CNT_PER_REV;

    /* a new visible half-sector starts */
    if(phase_prev >= VISIBLE_HALF_CNT && phase < VISIBLE_HALF_CNT) {
        half_count++;
        if(half_count >= LETTER_HOLD_HALVES) {
            half_count = 0;
            letter_index++;
            if(letter_index >= 5) letter_index = 0;
        }
    }
    phase_prev = phase;

    if(phase < VISIBLE_HALF_CNT) {
        col = (int32_t)phase * 18 / VISIBLE_HALF_CNT;
        if(col > 17) col = 17;
        if(col != last_col) {
            last_col = (int8_t)col;
            HC595_Write(g_columns[letter_index][(uint8_t)col]);
        }
    } else {
        if(last_col != -1) {
            HC595_Write(0);
            last_col = -1;
        }
    }
}

/* ===================== Peripherals ===================== */
void RCC_Config(void)
{
    RCC_APB2PeriphClockCmd(
        RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,
        ENABLE);
    RCC_APB1PeriphClockCmd(
        RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4,
        ENABLE);
}

void GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* Motor PWM: PA6(TIM3_CH1), PA7(TIM3_CH2) */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Encoder: PA0, PA1 = TIM2 CH1, CH2 */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Driver enable PB0 (harmless if module has no EN pin) */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void TIM3_PWM_Config(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    TIM_OCInitTypeDef TIM_OCInitStruct;

    TIM_TimeBaseStruct.TIM_Period = PWM_MAX - 1;   /* 10kHz */
    TIM_TimeBaseStruct.TIM_Prescaler = 0;
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStruct);

    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_Pulse = 0;
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM3, &TIM_OCInitStruct);
    TIM_OC2Init(TIM3, &TIM_OCInitStruct);

    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
}

void TIM2_Encoder_Config(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;

    TIM_TimeBaseStruct.TIM_Period = 65535;
    TIM_TimeBaseStruct.TIM_Prescaler = 0;
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStruct);

    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising,
                               TIM_ICPolarity_Rising);
    TIM_SetCounter(TIM2, 0);
    TIM_Cmd(TIM2, ENABLE);
}

void TIM4_Sample_Config(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    TIM_TimeBaseStruct.TIM_Period = 100 - 1;
    TIM_TimeBaseStruct.TIM_Prescaler = 7200 - 1;
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStruct);

    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

    NVIC_InitStruct.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    TIM_Cmd(TIM4, ENABLE);
}

void TIM4_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
        g_10msFlag = 1;
    }
}

/* ===================== Main ===================== */
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    RCC_Config();
    GPIO_Config();
    LED_Init();
    BuildColumns();
    TIM3_PWM_Config();
    TIM2_Encoder_Config();
    TIM4_Sample_Config();

    /* virtual zero: count starts at 0 while the rotor is held still */
    TIM_SetCounter(TIM2, 0);

    GPIO_SetBits(GPIOB, GPIO_Pin_0);
    HC595_Write(0);

    while(1) {
        UpdateEncoderPosition();

        if(g_10msFlag) {
            g_10msFlag = 0;
            Motor_Control();
        }

        Display_Update();
    }
}
