#include "config.h"
#include "stm32f767xx.h"
#include "clock.h"
#include "GPIO.h"
#include "adc.h"
#include "timer.h"
#include "hall.h"
#include "uart.h"
#include "dac.h"

#define MAX3(a, b, c) (((a) > (b)) ? (((a) > (c)) ? (a) : (c)) : (((b) > (c)) ? (b) : (c)))  // 3상 전류 중 최대 전류 검출, 과전류 보호용
#define Tsamp	0.00005				// sampling time of current controller(50us(20kHz)), TIM1 PWM 주기 일치

void TIM1_UP_TIM10_IRQHandler(void);		/* TIM1 interrupt function(10kHz) */
void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
void EXTI2_IRQHandler(void);
void SpeedCal(void);
void Update_Hall_Sequence(void);
void rampToTarget(float command, float *output, float slope);

float Vdc = 0.0f;
float MosfetTemp = 0.0f;
volatile float calculated_rpm =0.0f;		// Hall 엣지 기반 즉시 RPM
volatile float motor_speed_rpm = 0.0f;		// LPF 적용된 실제 제어용 RPM
float Fi, Fv, Ft=0.0f;				// digital low-pass filter coefficient
float I_Max = 0.0f;
float tempLaw =0.0f;
volatile float speed_km_h = 0.0f;
float ias_LPF, ibs_LPF, ics_LPF = 0.0f;
float OpenLoopTestRef=0.0f, OpenLoopRampOut=0.0f, ThrottleRef =0.0f, ThrottleRef_Ramp=0.0f;
float Volt = 0.0f,Throttle_ADC = 0.0f;

float ias,ibs,ics=0.0f;
float iRamp = 0.15f;
float ias_Cal,ibs_Cal,ics_Cal =0.0f;

volatile uint32_t last_hall_time = 0;
volatile uint32_t msTicks = 0;
uint32_t VoltageRef = 0;
uint32_t Tim1TestCnt =0;
uint32_t Ias_Offset,Ibs_Offset,Ics_Offset=0;
volatile uint32_t delta_time =0;
uint8_t Task_10msFlg,Task_100msFlg,Task_1sFlg,Task_500msFlg = 0;
uint8_t HA,HB,HC = 0;		// Hall 센서 입
uint8_t HallSum = 0;		// HallSum = HA*4 + HB*2 + HC
uint8_t StartFlag = 1;		// 구동 허용
uint8_t InitCal = 0;		// 초기 보정 완료 여부
uint8_t FltFlg = 0;			// Fault 상태
uint8_t ThrottleActive = 0;
uint8_t FltCnt = 0;
uint8_t init_drive = 0;

static const float Edges_per_Revolution = HALL_EDGES_PER_REV;
extern uint32_t DutyA,DutyB,DutyC;

void rampToTarget(float command, float *output, float slope)
{
    // 지령 신호에 따라 출력 신호를 점진적으로 변화시킴
    if(*output < command)				// 급가속 방지
    {
        *output += slope;

        if(*output > command)
        {
            *output = command;
        }
    }
    else if(*output > command)			// 급가속 방지
    {
        *output -= slope;

        if(*output < command)
        {
            *output = command;
        }
    }
}

void LPF(float input, float Fx, float *output)	/* digital low-pass filter */
{
  *output = (1. - Fx)*(*output) + Fx*input;		// 1차 IIR 필터, 노이즈 제거, 전류/속도 안정화
}
float RpmRef,RpmErr,Pterm,Iterm,PIterm = 0.0f;
float Kp,Ki = 0.0f;

uint8_t SpdFlg = 0;

void Task_1ms(void)
{
	if(SpdFlg==1)
	{
	    RpmErr = RpmRef-motor_speed_rpm;
	    Pterm = Kp*RpmErr;
	    Iterm += Ki*RpmErr*0.001f; // 0.001--> PI 속도제어기가 실행되는 주기
	    PIterm = Pterm + Iterm;

	    if(PIterm>CNT_MAX-100)
	    {
	    	PIterm = CNT_MAX-100;
	    }

	    VoltageRef = PIterm;
	}
	else
	{
		Pterm = 0.0f;
		Iterm = 0.0f;
		PIterm = 0.0f;
	}
}

/* 10ms마다 실행할 태스크 */
void Task_10ms(void)
{
	uint32_t result=0;
    // PA6(tempLaw) 읽기 - ADC1

    ADC1->SQR3 = 0x00000006;  // SQ1=6 (채널 6), 온도
    ADC1->CR2 |= 0x40000000;  // 변환 시작
    while(!(ADC1->SR & 0x00000002));  // EOC 대기
    result = ADC1->DR;
    tempLaw = (float)result*ADC_VREF / ADC_FS;

    //y = -11.489x3 + 63.236x2 - 149.02x + 181.97 NTC 3차 방정식
    MosfetTemp = -11.48f*tempLaw*tempLaw*tempLaw+63.23*tempLaw*tempLaw-149.02*tempLaw+181.97f;  // NTC 서미스터 보정식

    // PA3(Vdc) 읽기 - ADC3
    ADC3->SQR3 = 0x00000003;  // SQ1=3 (채널 3), DC 전압
    ADC3->CR2 |= 0x40000000;
    while(!(ADC3->SR & 0x00000002));
    result = ADC3->DR;
    Vdc = (float)result*(ADC_VREF/ADC_FS) / VDIV_RATIO;

//    // 온도, 전압 기반 폴트 처리
    if(Vdc < 32.0f)
    {
//        FltFlg = 3;  // 저전압 감지, 인휠모터 구동 시 주석 해제
    }

    if(MosfetTemp > 100.0f)
    {
        FltFlg = 2;  // 과열 감지
    }
    else if(MosfetTemp < 90.0f && FltFlg == 2)
    {
        FltFlg = 0;  // 온도가 정상이면 폴트 클리어
    }

	UART2_SendString(">RpmRef:");
    UART2_SendFloat_Simple(RpmRef,1);
	UART2_SendString("\n");

	UART2_SendString(">RpmFdb:");
    UART2_SendFloat_Simple(motor_speed_rpm,1);
	UART2_SendString("\n");

    Task_10msFlg=0;
}

/* 100ms마다 실행할 태스크 */
void Task_100ms(void)
{
	Task_100msFlg=0;
}

/* 500ms마다 실행할 태스크 */
void Task_500ms(void)
{
	// 블루투스 송신
	USART3_SendString("Spd :");
	UART3_SendFloat_Simple(speed_km_h,1);
	USART3_SendString("\n");

	USART3_SendString("Vdc :");
	UART3_SendFloat_Simple(Vdc,1);
	USART3_SendString("\n");

	USART3_SendString("MosfetTemp :");
	UART3_SendFloat_Simple(MosfetTemp,1);
	USART3_SendString("\n");

	USART3_SendString("Flt :");
	USART3_SendChar(FltFlg+48);
	USART3_SendString("\n");

	Task_500msFlg=0;
}

/* 1초마다 실행할 태스크 */
void Task_1sec(void)
{
	Task_1sFlg = 0;
}

/* 스케줄러 함수: msTicks 값을 기준으로 태스크 호출 */
void Scheduler(void)
{
    // 1ms 태스크는 매번 실행
    Task_1ms();

    // 10ms 주기 태스크: 1ms 카운터가 10의 배수이면 실행
    if ((msTicks % 10) == 0)
    {
    	Task_10msFlg=1;
    }
    // 100ms 주기 태스크: 1ms 카운터가 100의 배수이면 실행
    if ((msTicks % 100) == 0)
    {
    	Task_100msFlg=1;
    }
    // 500ms 주기 태스크: 1ms 카운터가 1000의 배수이면 실행
    if ((msTicks % 500) == 0)
    {
    	Task_500msFlg=1;
    }
    // 1초 주기 태스크: 1ms 카운터가 1000의 배수이면 실행
    if ((msTicks % 1000) == 0)
    {
    	Task_1sFlg=1;
    }
}

/* SysTick 인터럽트 핸들러: 1ms마다 호출됨 */
void SysTick_Handler(void)
{
    msTicks++;
    Scheduler();
}

/* SysTick 초기화 함수: 1ms 주기로 인터럽트 발생 */
void SysTick_Init(void)
{
    // SYSCLK가 216MHz일 때, 1ms마다 인터럽트가 발생하도록 설정:
    // Reload = (216,000,000 / 1000) - 1 = 215999
    SysTick->LOAD = (216000000 / 1000) - 1;  // 1ms
    SysTick->VAL  = 0;  // 현재 카운터 값 초기화
    // SysTick 제어: 프로세서 클록(216MHz) 사용, 인터럽트 활성, 카운터 시작
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk   |
                    SysTick_CTRL_ENABLE_Msk;
}

uint32_t rpmHoldCounter = 0;
float RpmNew = 0.0f;
float RpmOld = 0.0f;
void TIM1_UP_TIM10_IRQHandler(void)
{
	Tim1TestCnt++;

    RpmNew = calculated_rpm;

    if(RpmNew==RpmOld)
    {
    	rpmHoldCounter++;
    	if(rpmHoldCounter>20000)
    	{
    		calculated_rpm = 0.0f;
    		rpmHoldCounter = 0;
    	}
    }
    RpmOld = calculated_rpm;

    if ((TIM1->SR & 0x0001) && InitCal==1)  // update interrupt flag ?
    {
    	uint32_t result = 0;

        TIM1->SR &= ~TIM_SR_UIF;

        // PA0(ias) 읽기 - ADC1
        ADC1->SQR3 = 0x00000000;  // SQ1=0 (채널 0)
        ADC1->CR2 |= 0x40000000;  // SWSTART 비트 설정
        while(!(ADC1->SR & 0x00000002));  // EOC 대기
        result = ADC1->DR;        // 변환 결과 읽기
        ias_Cal=((float)(result - Ias_Offset)*ADC_VREF/ADC_FS-OFFSET_Volt)/OPAMP_GAIN;
        ias = ias_Cal;
        LPF(ias, Fi, &ias_LPF);

        // PA1(ibs) 읽기 - ADC2
        ADC2->SQR3 = 0x00000001;  // SQ1=1 (채널 1)
        ADC2->CR2 |= 0x40000000;
        while(!(ADC2->SR & 0x00000002));
        result = ADC2->DR;
        ibs_Cal=((float)(result - Ibs_Offset)*ADC_VREF/ADC_FS-OFFSET_Volt)/OPAMP_GAIN;
        ibs = ibs_Cal;
        LPF(ibs, Fi, &ibs_LPF);

        // PA2(ics) 읽기 - ADC3
        ADC3->SQR3 = 0x00000002;  // SQ1=2 (채널 2)
        ADC3->CR2 |= 0x40000000;
        while(!(ADC3->SR & 0x00000002));
        result = ADC3->DR;
        ics_Cal=((float)(result - Ics_Offset)*ADC_VREF/ADC_FS-OFFSET_Volt)/OPAMP_GAIN;
        ics = ics_Cal;
        LPF(ics, Fi, &ics_LPF);

        // PA7(Throttle_ADC) 읽기 - ADC2
        ADC2->SQR3 = 0x00000007;  // SQ1=7 (채널 7)
        ADC2->CR2 |= 0x40000000;
        while(!(ADC2->SR & 0x00000002));
        result = ADC2->DR;
        Throttle_ADC = (float)result*3.3f/4095.0f;

        // 최대 전류 계산 및 과전류 보호
        I_Max = MAX3(ias, ibs, ics);

        if(I_Max > OC_LEVEL)
        {
            FltCnt++;
            if(FltCnt >= 1000) //50ms동안 확인
            {
                FltFlg = 1;
                ThrottleRef = 0;
            }
        }
        else if(I_Max < 30.0f)
        {
        	FltCnt = 0;
        }

        // 속도 계산 *********************수정해야함
        LPF(calculated_rpm, Ft, &motor_speed_rpm);
        speed_km_h = RPM_TO_KMH(motor_speed_rpm);

        // 히스테리시스를 주어 쓰로틀 신호 제어
        if (Throttle_ADC < THROTTLE_OFF)
        {
            ThrottleActive = 0;
        }
        else if (Throttle_ADC > THROTTLE_ON)
        {
            ThrottleActive = 1;
        }

		// 6-STEP 제어 모드
		if (!ThrottleActive) //히스테리시스 값 이하
		{
			Disable_PWM();
			ThrottleRef = 0.0f;
			VoltageRef = 0;
		}
		else // 모터 구동
		{
			ThrottleRef = Throttle_ADC * 3400.0f - 3540.0f+1000.0f; // 0826 듀티값 변경
		}

		rampToTarget(ThrottleRef, &ThrottleRef_Ramp, iRamp); // Ramp 함수를 통한 모터 응답성 조절

		if(SpdFlg == 0)
		{
			VoltageRef = (uint32_t)ThrottleRef_Ramp; // 형 변환을 통한 최종 지령 값 추출

			if(VoltageRef> CNT_MAX-100)
			{
				VoltageRef = CNT_MAX-100;
			}
		}

        if(FltFlg == 0 && InitCal == 1)
		{
			Update_Switching_Pattern(HallSum);  // Hall -> 상전환, 실제 모터 구동
		}
		else
		{
			VoltageRef = 0;
			DutyA = 0;
			DutyB = 0;
			DutyC = 0;
			// PWM 완전 차단
			TIM1->CCR1 = 0;
			TIM1->CCR2 = 0;
			TIM1->CCR3 = 0;
		}
    }
}
void EXTI0_IRQHandler(void) //PORTD0 HA
{
    if (EXTI->PR & EXTI_PR_PR0)
    {
        EXTI->PR |= EXTI_PR_PR0; // 인터럽트 플래그 클리어
        Update_Hall_Sequence();
        SpeedCal();
    }
}

void EXTI1_IRQHandler(void) //PORTD1 HB
{
    if (EXTI->PR & EXTI_PR_PR1)
    {
        EXTI->PR |= EXTI_PR_PR1; // 인터럽트 플래그 클리어
        Update_Hall_Sequence();
        SpeedCal();
    }
}

void EXTI2_IRQHandler(void) //PORTD2 HC
{
	if (EXTI->PR & EXTI_PR_PR2)
	{
        EXTI->PR |= EXTI_PR_PR2; // 인터럽트 플래그 클리어
        Update_Hall_Sequence();
        SpeedCal();
    }
}

int main(void)
{
	Initialize_MCU();
	Initialize_ADC();				// initialize ADC for measurement
	Initialize_PWM();				// initialize TIM1 for PWM
	Initialize_TIM2();
	FLT_LED_Init();
	Initialize_Hall_Sensors();
	AT09_Init();
	SysTick_Init();
	DAC_Init();
	UART2_Init();

	Fi = 2.*PI*500.*Tsamp/(1.+2.*PI*500.*Tsamp);	// fci = 1000[Hz] for phase current
	Ft = 2.*PI*1.*Tsamp/(1.+2.*PI*1.*Tsamp);	// fct = 100[Hz] for IPM temperature

	for(int i = 0; i < 10; i++)
	{
	    // PA0(ias) 읽기 - ADC1
	    ADC1->SQR3 = 0x00000000;  // SQ1=0 (채널 0)
	    ADC1->CR2 |= 0x40000000;
	    while(!(ADC1->SR & 0x00000002));
	    Ias_Offset += ADC1->DR;

	    // PA1(ibs) 읽기 - ADC2
	    ADC2->SQR3 = 0x00000001;  // SQ1=1 (채널 1)
	    ADC2->CR2 |= 0x40000000;
	    while(!(ADC2->SR & 0x00000002));
	    Ibs_Offset += ADC2->DR;

	    // PA2(ics) 읽기 - ADC3
	    ADC3->SQR3 = 0x00000002;  // SQ1=2 (채널 2)
	    ADC3->CR2 |= 0x40000000;
	    while(!(ADC3->SR & 0x00000002));
	    Ics_Offset += ADC3->DR;
	}

	// 평균 오프셋 계산
	Ias_Offset = (Ias_Offset/10) - 2048;
	Ibs_Offset = (Ibs_Offset/10) - 2048;
	Ics_Offset = (Ics_Offset/10) - 2048;

	if(FltFlg==1)
	{
		InitCal = 0;
	}
	else
	{
		InitCal = 1;
	}

	NVIC->ISER[0] |= 0x02000000;			// enable (25)TIM1 update Interrupt
	while(!(TIM1->CR1 & 0x0010));                 // TIM1 underflow event ?
	TIM1->RCR = 0x0001;                           // 100 us period update(RCR = 1)

	// 모터가 정지되어 있는 상태에서는 엣지 검출을 통한 홀센서 신호를 검출할 수 없으므로 초기 위치 필요
	HA = GPIOD->IDR & GPIO_IDR_ID0;
	HB = (GPIOD->IDR & GPIO_IDR_ID1) >> 1;
	HC = (GPIOD->IDR & GPIO_IDR_ID2) >> 2;
	HallSum = HA*2*2+HB*2*1+HC;

  while(1)
    {

	  if(Task_1sFlg==1)
	  {
		  Task_1sec();
	  }
	  else if(Task_10msFlg==1)
	  {
		  Task_10ms();
	  }
	  else if(Task_100msFlg==1)
	  {
		  Task_100ms();
	  }
	  else if(Task_500msFlg==1)
	  {
		  Task_500ms();
	  }
	  else
	  {

	  }

	  if(StartFlag==1)//StartFlag 초기값 1, 개발 테스트 시 사용
	  {
		  Enable_PWM();
		  if(init_drive==0)
		  {
			 init_drive=1;
		  }
	  }
	  else
	  {
		  Disable_PWM();
		  init_drive=0;
	  }

    }
//while End
  }

void SpeedCal(void)
{
    // 현재 타이머 값 읽기
    volatile uint32_t current_time = TIM2->CNT;

    // 타이머 오버플로우 처리
    if (current_time >= last_hall_time)
    {
        delta_time = current_time - last_hall_time;
    }
    else
    {
        // 오버플로우 발생: 타이머가 ARR 값에 도달하여 리셋됨
        delta_time = (TIM2->ARR - last_hall_time) + current_time + 1;
    }

    last_hall_time = current_time;

    // delta_time이 0이 아닐 때만 RPM 계산
        // 가정: 한 회전당 6개의 홀 센서 이벤트 발생 (6 edges per revolution)
        // Timer 주파수: 54MHz
        // RPM 계산 공식: RPM = (60 * Clock_Frequency) / (Edges_per_Revolution * delta_time)
    if(delta_time <= 500)
    {
    	//너무 빠른 신호는 무시
    }
    else
    {
    	calculated_rpm = (60.0f * 54000000.0f) / ((Edges_per_Revolution * (float)delta_time));
    }
}

void Update_Hall_Sequence(void)
{
    // PD0, PD1, PD2에서 홀 신호 읽기
    HA = GPIOD->IDR & GPIO_IDR_ID0;
    HB = (GPIOD->IDR & GPIO_IDR_ID1) >> 1;
    HC = (GPIOD->IDR & GPIO_IDR_ID2) >> 2;
	HallSum = HA*2*2+HB*2*1+HC;
}
