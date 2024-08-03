#include "stm32f4xx.h"
#include "stm32f4_discovery.h"

// Structure type definitions
GPIO_InitTypeDef GPIO_InitStruct;
TIM_TimeBaseInitTypeDef TIM_InitStruct;
TIM_OCInitTypeDef TIM_OCInitStruct;
ADC_CommonInitTypeDef ADC_CommonInitStruct;
ADC_InitTypeDef ADC_InitStruct;
NVIC_InitTypeDef NVIC_InitStruct;
EXTI_InitTypeDef EXTI_InitStruct;

uint8_t adc_value; // Variable to hold the ADC value
int button_count = 0; // Variable to count button presses
uint32_t delay_count; // Variable to hold delay count

// Millisecond delay function
void delay_ms(uint32_t time) {
    delay_count = time;
    while (delay_count);
}

// SysTick interrupt handler
void SysTick_Handler(void) {
    delay_count--;
}

// GPIO pin configuration
void GPIO_config() {
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    // Configure GPIOD pins as output
    GPIO_InitStruct.GPIO_Mode = 0x01;
    GPIO_InitStruct.GPIO_OType = 0x00;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_InitStruct.GPIO_PuPd = 0x00;
    GPIO_InitStruct.GPIO_Speed = 0x03;
    GPIO_Init(GPIOD, &GPIO_InitStruct);

    // Button pin configuration (PA0)
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // PA5 as an analog input for ADC1
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // PA2 as AF for PWM output (TIM2 CH3)
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2);
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Configure PA3 and PA4 pins as output
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
}

// ADC configuration
void ADC_config() {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStruct.ADC_Prescaler = ADC_Prescaler_Div4;
    ADC_CommonInit(&ADC_CommonInitStruct);

    ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStruct.ADC_Resolution = ADC_Resolution_8b;
    ADC_Init(ADC1, &ADC_InitStruct);
    ADC_Cmd(ADC1, ENABLE);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1 , ADC_SampleTime_56Cycles);
    ADC_SoftwareStartConv(ADC1);
}

// Function to read ADC value
uint8_t read_adc() {
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
    return ADC_GetConversionValue(ADC1);
}

// Timer configuration for PWM
void TIM_config() {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_InitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_InitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_InitStruct.TIM_Period = 255;
    TIM_InitStruct.TIM_Prescaler = 299;
    TIM_InitStruct.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(TIM2, &TIM_InitStruct);
    TIM_Cmd(TIM2, ENABLE);

    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;

    SysTick_Config(SystemCoreClock / 1000); // 1ms interrupt
}

// EXTI configuration for button interrupt
void EXTI_config() {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

    EXTI_InitStruct.EXTI_Line = EXTI_Line0;
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;

    EXTI_Init(&EXTI_InitStruct);

    NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;

    NVIC_Init(&NVIC_InitStruct);
}

// EXTI0 interrupt handler for button press
void EXTI0_IRQHandler() {
    if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
        button_count++;
        if (button_count % 2 == 0) {
            GPIO_SetBits(GPIOA, GPIO_Pin_3);
            GPIO_ResetBits(GPIOA, GPIO_Pin_4);
            GPIO_SetBits(GPIOD, GPIO_Pin_12);
            GPIO_ResetBits(GPIOD, GPIO_Pin_13);
        } else {
            GPIO_SetBits(GPIOA, GPIO_Pin_4);
            GPIO_ResetBits(GPIOA, GPIO_Pin_3);
            GPIO_SetBits(GPIOD, GPIO_Pin_13);
            GPIO_ResetBits(GPIOD, GPIO_Pin_12);
        }
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}

int main(void) {
    GPIO_config();
    ADC_config();
    TIM_config();
    EXTI_config();
    GPIO_SetBits(GPIOA, GPIO_Pin_3);
    GPIO_ResetBits(GPIOA, GPIO_Pin_4);
    GPIO_SetBits(GPIOD, GPIO_Pin_12);
    GPIO_ResetBits(GPIOD, GPIO_Pin_13);

    while (1) {
        adc_value = read_adc();
        TIM_OCInitStruct.TIM_Pulse = adc_value;
        TIM_OC3Init(TIM2, &TIM_OCInitStruct);
        TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
    }
}

void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size) {
    /* TODO, implement your code here */
    return;
}

/*
 * Callback used by stm324xg_eval_audio_codec.c.
 * Refer to stm324xg_eval_audio_codec.h for more info.
 */
uint16_t EVAL_AUDIO_GetSampleCallBack(void) {
    /* TODO, implement your code here */
    return -1;
}
