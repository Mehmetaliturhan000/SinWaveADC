#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "math.h"
#include "delay.h"

#define M_PI 3.14159265358979323846

uint32_t sayac = 0;
uint32_t adc_value = 0;
uint32_t freq = 0;
uint32_t led1_value = 0;
uint32_t led2_value = 0;
uint32_t led3_value = 0;
uint32_t led4_value = 0;

void LEDS_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct;
	
	/* Enable clock for GPIOD */
	//***********
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	//***********

	/* Alternating functions for pins */
	//***********
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);
	//***********
	
	/* Set pins */
	//***********
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14  | GPIO_Pin_15;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;   //All chosen push pull
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; //All chosen as nopull
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;     //Alternating function to use PWM 
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;   //Full speed
  GPIO_Init(GPIOD, &GPIO_InitStruct);               //Load the configuration
	//***********
}

void PWM_TIMER_Init(void) {
	TIM_TimeBaseInitTypeDef TIM_BaseStruct;
	
	
	/* Enable clock for TIM4 */
	//***********
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	//***********
	 
	/* TIM4 Base Settings */                    																				
	//*********** 										           
	TIM_BaseStruct.TIM_Prescaler = 0;   
  TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_BaseStruct.TIM_Period = 36000;
  TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_BaseStruct.TIM_RepetitionCounter = 0;
	//***********
	
	/* Initialize TIM4 */
	//***********
  TIM_TimeBaseInit(TIM4, &TIM_BaseStruct);
	//***********
	
	/* Start count on TIM4 */
  TIM_Cmd(TIM4, ENABLE);
	 
}


void counter_TIMER_Init()
{
  TIM_TimeBaseInitTypeDef TIM_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);	 	  //TIMER TIM1 clock enable
	TIM_InitStruct.TIM_Prescaler = 249;												  //Prescaler set is 249	
	TIM_InitStruct.TIM_CounterMode = TIM_CounterMode_Up;  	  //Counter mode up
	TIM_InitStruct.TIM_Period = 63999;							// I used this formula to calculate the period--->Update Event = TIM clk/((PSC+1)*(ARR+1)*(RCR+1)) 
	TIM_InitStruct.TIM_ClockDivision = 0;            // however it missed with small numbers  than I incremented that value a little bit experimentally. 
	TIM_InitStruct.TIM_RepetitionCounter = 0;                 //Repetition counter set 0.
	
	TIM_TimeBaseInit(TIM1,&TIM_InitStruct);                   //Load the configuration
	
	
	TIM_Cmd(TIM1,ENABLE);																		  //TIM1 enable
	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);                  //Configures the TIMx event by the software
	
	NVIC_InitStruct.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;     //NVIC channel selected.
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority  = 0;   //NVIC Preemption Priority set as 0 
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;						//NVIC Sub Priority set as 0 
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;              //NVIC Channel enabled when interrupt comes
  
	NVIC_Init(&NVIC_InitStruct);                              //Load
	
}	


void ADC_Config() 
{
	GPIO_InitTypeDef GPIO_InitStruct;                    //GPIO init struct defined for pin.
	ADC_InitTypeDef ADC_InitStruct;                      //ADC init struct defined.
	ADC_CommonInitTypeDef ADC_CommonInitStruct;          //ADC common init struct defined.
	 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //Port A clock opened, I used Port A pin 0.
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);  //ADC clock opened.
	
	 
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;            //For adc usage, analog mode chosen.
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;               //Pin 0 chosen.
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;          //Output type set as push pull.
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;        // No pull configuration set.
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;      // Full speed
  GPIO_Init(GPIOA, &GPIO_InitStruct);                  //Load the configuation
	
	ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;   //Independent mode chosen, (we are going to read only 1 channel).
	ADC_CommonInitStruct.ADC_Prescaler =  ADC_Prescaler_Div4; //Div 4 chosen for clock, APB2 line divided to 4.
	
  ADC_CommonInit(&ADC_CommonInitStruct);               //Load the configuration
	
	ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;    //Resolution set 12bit
	ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;        //For contiunuous analog read it's set to enable.
	ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;   //We don't need a trigger so none selected
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Left;    //Data aligned to left.
	
	
	ADC_Init(ADC1,&ADC_InitStruct);     //Configuration loaded 
	
	ADC_Cmd(ADC1,ENABLE);               //ADC start command.
	
}

uint32_t read_ADC()
{
	ADC_RegularChannelConfig(ADC1,ADC_Channel_0,1,ADC_SampleTime_56Cycles);
	
	ADC_SoftwareStartConv(ADC1);
	
	while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC == RESET));
	
	return ADC_GetConversionValue(ADC1);
	
}	



void TIM1_UP_TIM10_IRQHandler()
{
	if(TIM_GetITStatus(TIM1,TIM_IT_Update) == SET)
	{
	
	sayac = sayac + 1;
		if (sayac == 60)
		{
			sayac = 0;
		}
			
		}
		TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
	}




int main(void) {
	/* Initialize system */
	SystemInit();
	/* Init leds */
	
	LEDS_Init();
	/* Init timer */

	PWM_TIMER_Init();
	/* Init PWM */
		
	ADC_Config();
	/* Init ADC */
	
	counter_TIMER_Init();
	/* Init counter timer */
	
	TIM_OCInitTypeDef TIM_OCStruct;           // Timer struct defined for pwm.
	TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM2;  //PWM2 mode selected
	TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;  //Output state enable
	TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_Low;       //Polarity chosen as low
	
	
	while (1) {
  adc_value = read_ADC();    //Adc read called
	freq = adc_value+2000;	         // Freq map between 2000 4096
		
	led1_value = 36000*(sin(2*22/7*freq*sayac-1.5)+1)/2;      //led output values
	led2_value = 36000*(sin(2*22/7*freq*sayac-0.5)+1)/2;      //I multipy these values with 8399
	led3_value = 36000*(sin(2*22/7*freq*sayac+0.5)+1)/2;      // to see in PWM output.
  led4_value = 36000*(sin(2*22/7*freq*sayac+1.5)+1)/2;      //PWM period value was 8399
	
//	led1_value = sayac*adc_value;
//	led2_value = sayac*adc_value;
//	led3_value = sayac*adc_value;
//  led4_value = sayac*adc_value;	
	
				  	 
	TIM_OCStruct.TIM_Pulse = led1_value;     
	TIM_OC1Init(TIM4, &TIM_OCStruct);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
	TIM_OCStruct.TIM_Pulse = led2_value; 
	TIM_OC2Init(TIM4, &TIM_OCStruct);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
	TIM_OCStruct.TIM_Pulse = led3_value; 
	TIM_OC3Init(TIM4, &TIM_OCStruct);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
		
	TIM_OCStruct.TIM_Pulse = led4_value; 
	TIM_OC4Init(TIM4, &TIM_OCStruct);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);	
}
}
