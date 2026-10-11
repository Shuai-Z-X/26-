#include "stm32f10x.h"

// ================== 引脚定义（根据实际接线修改） ==================
#define DS_PORT     GPIOA
#define DS_PIN      GPIO_Pin_0    // SER / DS  -> 595 的 14 脚
#define SHCP_PORT   GPIOA
#define SHCP_PIN    GPIO_Pin_1    // SCK / SHCP -> 595 的 11 脚
#define STCP_PORT   GPIOA
#define STCP_PIN    GPIO_Pin_2    // RCK / STCP -> 595 的 12 脚

// ================== 底层驱动函数 ==================

static void delay_us(uint32_t n) {
    for(volatile uint32_t i = 0; i < n * 8; i++);
}

static void delay_ms(uint32_t ms) {
    for(volatile uint32_t i = 0; i < ms; i++) delay_us(1000);
}

// SHCP 脉冲（上升沿移位）
static void HC595_SHCP_Pulse(void) {
    GPIO_SetBits(SHCP_PORT, SHCP_PIN);
    delay_us(1);
    GPIO_ResetBits(SHCP_PORT, SHCP_PIN);
    delay_us(1);
}

// STCP 脉冲（上升沿锁存）
static void HC595_STCP_Pulse(void) {
    GPIO_SetBits(STCP_PORT, STCP_PIN);
    delay_us(1);
    GPIO_ResetBits(STCP_PORT, STCP_PIN);
    delay_us(1);
}

// 发送一个字节（MSB 先发：bit7 最终到 Q7，bit0 最终到 Q0）
static void HC595_SendByte(uint8_t data) {
    for(int i = 0; i < 8; i++) {
        if(data & 0x80) GPIO_SetBits(DS_PORT, DS_PIN);
        else            GPIO_ResetBits(DS_PORT, DS_PIN);
        HC595_SHCP_Pulse();
        data <<= 1;
    }
}

// 发送 3 字节到级联 595
// 级联关系：MCU → U3(第一级) → U4(第二级) → U5(第三级)
// 发送顺序：先发 U5（第三级），再发 U4（第二级），最后发 U3（第一级）
// 映射关系：
//   bit0~7   -> U3 Q0~Q7（第一级，最后发，留在 U3）
//   bit8~15  -> U4 Q0~Q7（第二级，中间发，留在 U4）
//   bit16~17 -> U5 Q0~Q1（第三级，先发，最终到 U5）
void HC595_Write(uint32_t mask) {
    uint8_t u3 = (mask >> 0)  & 0xFF;   // U3(第一级): Q0~Q7 = bit0~7
    uint8_t u4 = (mask >> 8)  & 0xFF;   // U4(第二级): Q0~Q7 = bit8~15
    uint8_t u5 = (mask >> 16) & 0xFF;   // U5(第三级): Q0~Q7 = bit16~23
    
    HC595_SendByte(u5);  // 先发：经过 U3→U4，最终到达 U5（第三级）
    HC595_SendByte(u4);  // 中间发：经过 U3，最终到达 U4（第二级）
    HC595_SendByte(u3);  // 最后发：留在 U3（第一级）
    HC595_STCP_Pulse();  // 锁存，全部输出更新
}

// ================== GPIO 初始化 ==================
void HC595_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    GPIO_InitStruct.GPIO_Pin = DS_PIN | SHCP_PIN | STCP_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DS_PORT, &GPIO_InitStruct);
    
    GPIO_ResetBits(DS_PORT, DS_PIN);
    GPIO_ResetBits(SHCP_PORT, SHCP_PIN);
    GPIO_ResetBits(STCP_PORT, STCP_PIN);
    
    HC595_Write(0);  // 上电全灭
    delay_ms(50);
}

// ================== 测试函数 ==================

// 模式1：逐位扫描（U3 Q0→Q7 → U4 Q0→Q7 → U5 Q0→Q1）
// 用于验证级联顺序和硬件接线
void Test_Scan(void) {
    while(1) {
        for(int i = 0; i < 18; i++) {
            HC595_Write(1UL << i);
            delay_ms(300);
        }
        HC595_Write(0);
        delay_ms(500);
    }
}

// 模式2：流水灯（从 U3 Q0 跑向 U5 Q1）
void Test_Flow(void) {
    while(1) {
        for(int i = 0; i < 18; i++) {
            HC595_Write(1UL << i);
            delay_ms(100);
        }
    }
}

// 模式3：按芯片逐位亮（先 U3 扫一遍，再 U4，再 U5）
void Test_ChipByChip(void) {
    while(1) {
        // U3(第一级) Q0~Q7
        for(int i = 0; i < 8; i++) {
            HC595_Write(1UL << i);
            delay_ms(200);
        }
        delay_ms(300);
        
        // U4(第二级) Q0~Q7
        for(int i = 0; i < 8; i++) {
            HC595_Write(1UL << (8 + i));
            delay_ms(200);
        }
        delay_ms(300);
        
        // U5(第三级) Q0~Q1
        for(int i = 0; i < 2; i++) {
            HC595_Write(1UL << (16 + i));
            delay_ms(200);
        }
        delay_ms(300);
    }
}

// 模式4：全亮全灭闪烁（测试电源/退耦）
void Test_Blink(void) {
    while(1) {
        HC595_Write(0x0003FFFF);  // 18 位全 1
        delay_ms(500);
        HC595_Write(0);
        delay_ms(500);
    }
}

// 模式5：二进制计数（快速验证所有 bit）
void Test_Count(void) {
    uint32_t cnt = 0;
    while(1) {
        HC595_Write(cnt & 0x0003FFFF);
        delay_ms(50);
        cnt++;
    }
}

// ================== 主函数 ==================
int main(void) {
    HC595_Init();
    
    // 建议先用 Test_Scan() 或 Test_ChipByChip() 验证级联顺序
    Test_Scan();
    // Test_ChipByChip();
    // Test_Flow();
    // Test_Blink();
    // Test_Count();
}