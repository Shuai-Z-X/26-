#include "stm32f10x.h"
#include "Delay.h"

/* PWM: 72MHz / 7200 = 10kHz */
#define PWM_PERIOD     7200
#define PWM_FULL_CCR   7200

void UART1_SendByte(uint8_t byte);
void VOFA_SendFloat(float value);

void UART1_SendByte(uint8_t byte)
{
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    USART_SendData(USART1, byte);
}

/* JustFloat: little-endian float + tail 00 00 80 7F */
void VOFA_SendFloat(float value)
{
    uint8_t *p = (uint8_t *)&value;
    const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};
    uint8_t i;

    for(i = 0; i < 4; i++) {
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, p[i]);
    }
    for(i = 0; i < 4; i++) {
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, tail[i]);
    }
}

int main(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    uint16_t duty = 0;
    int8_t dir = 1;

    /* Clocks */
    RCC_APB2PeriphClockCmd(
        RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO | RCC_APB2Periph_USART1,
        ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* PA6, PA7 = TIM3 CH1, CH2 PWM output */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* PA9 = USART1_TX */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART1 115200, TX only */
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);

    /* TIM3 PWM 10kHz */
    TIM_TimeBaseStructure.TIM_Period = PWM_PERIOD - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);  /* CH1 -> PA6 */
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);  /* CH2 -> PA7 */
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);

    while(1) {
        /* PA6 duty sweeps 0..100%, PA7 stays 0 */
        TIM_SetCompare1(TIM3, (uint16_t)((uint32_t)duty * PWM_FULL_CCR / 100));
        TIM_SetCompare2(TIM3, 0);

        /* Send duty (%) to VOFA+, JustFloat */
        VOFA_SendFloat((float)duty);

        Delay_ms(10);  /* 100Hz update, one sweep ~2 seconds */

        if(dir == 1) {
            if(duty >= 100) { duty = 99; dir = -1; }
            else duty++;
        } else {
            if(duty <= 0) { duty = 1; dir = 1; }
            else duty--;
        }
    }
}
// EOF
