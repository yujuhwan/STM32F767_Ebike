
#include "adc.h"

void Initialize_ADC(void)
{
    // GPIO 초기화
    GPIOA->MODER = 0xA955FFFF;  // PA0-PA7 아날로그 모드
    GPIOA->AFR[1] = 0x00000000;
    GPIOA->AFR[0] = 0x00000000;
    GPIOA->ODR = 0x00000000;
    GPIOA->OSPEEDR = 0xFC000000;

    // ADC 클럭 활성화
    RCC->APB2ENR |= 0x00000700;  // ADC1, ADC2, ADC3 클럭 활성화

    // ADC 공통 설정 - 독립 모드
    ADC->CCR = 0x00000000;  // 독립 모드, ADCCLK = PCLK2/2

    // ADC 샘플링 시간 설정 (모든 채널 15 사이클)
    ADC1->SMPR2 = 0x00249249;
    ADC2->SMPR2 = 0x00249249;
    ADC3->SMPR2 = 0x00249249;

    // ADC1 설정 - PA0(IN0), PA6(IN6)
    ADC1->CR1 = 0x00000000;
    ADC1->CR2 = 0x00000000;
    ADC1->SQR1 = 0x00000000;  // 1개 변환 (L=0)
    ADC1->SQR3 = 0x00000000;  // SQ1=0 (채널 0)

    // ADC2 설정 - PA1(IN1), PA7(IN7)
    ADC2->CR1 = 0x00000000;
    ADC2->CR2 = 0x00000000;
    ADC2->SQR1 = 0x00000000;  // 1개 변환 (L=0)
    ADC2->SQR3 = 0x00000001;  // SQ1=1 (채널 1)

    // ADC3 설정 - PA2(IN2), PA3(IN3)
    ADC3->CR1 = 0x00000000;
    ADC3->CR2 = 0x00000000;
    ADC3->SQR1 = 0x00000000;  // 1개 변환 (L=0)
    ADC3->SQR3 = 0x00000002;  // SQ1=2 (채널 2)

    // ADC 활성화
    ADC1->CR2 |= 0x00000001;  // ADON = 1
    ADC2->CR2 |= 0x00000001;
    ADC3->CR2 |= 0x00000001;
}





