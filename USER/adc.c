#include "adc.h"	  
#include "timer.h"	
#include "uart.h"
#include "string.h"
ADC_HandleTypeDef ADC1_Handler;//ADC��� 

uint16_t adc1;
uint16_t adc2;
uint16_t adc3;
uint16_t adc4;
uint16_t adc5;

uint8_t adc_1_H;
uint8_t adc_1_L;
uint8_t adc_2_H;
uint8_t adc_2_L;
uint8_t adc_3_H;
uint8_t adc_3_L;
uint8_t adc_4_H;
uint8_t adc_4_L;
uint8_t adc_5_H;
uint8_t adc_5_L;
uint8_t aa=1;

char c[20]={0};

void MX_ADC1_Init(void)
{
//  ADC_MultiModeTypeDef multimode = {0};
//  ADC_ChannelConfTypeDef sConfig = {0};

//  ADC1_Handler.Instance = ADC1;
//  ADC1_Handler.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;//fen ping
//  ADC1_Handler.Init.Resolution = ADC_RESOLUTION_16B;//wei shu
//  ADC1_Handler.Init.ScanConvMode = ADC_SCAN_DISABLE;//��ɨ��ģʽ
//  ADC1_Handler.Init.EOCSelection = ADC_EOC_SINGLE_CONV;//Eoc�ж�
//  ADC1_Handler.Init.LowPowerAutoWait = DISABLE;
//  ADC1_Handler.Init.ContinuousConvMode = DISABLE;//����ת��ģʽ�ر�
//  ADC1_Handler.Init.NbrOfConversion = 1;//����������ͨ����1
//  ADC1_Handler.Init.DiscontinuousConvMode = DISABLE;
//  
//  ADC1_Handler.Init.ExternalTrigConv = ADC_SOFTWARE_START;//�������
//  ADC1_Handler.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;//��������
//  ADC1_Handler.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
//  ADC1_Handler.Init.Overrun = ADC_OVR_DATA_PRESERVED;
//  ADC1_Handler.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
//  ADC1_Handler.Init.OversamplingMode = DISABLE;

//	HAL_ADC_Init(&ADC1_Handler);  
//	
//  sConfig.Channel = ADC_CHANNEL_14;
//  sConfig.Rank = ADC_REGULAR_RANK_1;
//  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
//  sConfig.SingleDiff = ADC_SINGLE_ENDED;
//  sConfig.OffsetNumber = ADC_OFFSET_NONE;
//  sConfig.Offset = 0;

    ADC1_Handler.Instance=ADC1;
    ADC1_Handler.Init.ClockPrescaler=ADC_CLOCK_SYNC_PCLK_DIV4; 	//4��Ƶ��ADCCLK=PER_CK/4=64/4=16MHZ
    ADC1_Handler.Init.Resolution=ADC_RESOLUTION_16B;           	//16λģʽ
    ADC1_Handler.Init.ScanConvMode=DISABLE;                    	//��ɨ��ģʽ
    ADC1_Handler.Init.EOCSelection=ADC_EOC_SINGLE_CONV;       	//�ر�EOC�ж�
	  ADC1_Handler.Init.LowPowerAutoWait=DISABLE;					//�Զ��͹��Ĺر�				
    ADC1_Handler.Init.ContinuousConvMode=DISABLE;               //�ر�����ת��
    ADC1_Handler.Init.NbrOfConversion=1;                        //1��ת���ڹ��������� Ҳ����ֻת����������1 
    ADC1_Handler.Init.DiscontinuousConvMode=DISABLE;            //��ֹ����������ģʽ
    ADC1_Handler.Init.NbrOfDiscConversion=0;                    //����������ͨ����Ϊ0
    ADC1_Handler.Init.ExternalTrigConv=ADC_SOFTWARE_START;      //�������
    ADC1_Handler.Init.ExternalTrigConvEdge=ADC_EXTERNALTRIGCONVEDGE_NONE;//ʹ���������
		ADC1_Handler.Init.BoostMode=ENABLE;							//BOOTģʽ�ر�
		ADC1_Handler.Init.Overrun=ADC_OVR_DATA_OVERWRITTEN;			//���µ����ݵ�����ֱ�Ӹ��ǵ�������
		ADC1_Handler.Init.OversamplingMode=DISABLE;					//�������ر�
		ADC1_Handler.Init.ConversionDataManagement=ADC_CONVERSIONDATA_DR;  //����ͨ�������ݽ���������DR�Ĵ�������
    HAL_ADC_Init(&ADC1_Handler);                                 //��ʼ�� 

		HAL_ADCEx_Calibration_Start(&ADC1_Handler,ADC_CALIB_OFFSET,ADC_SINGLE_ENDED); //ADCУ׼
}

uint16_t Get_Adc(ADC_HandleTypeDef *hadc,uint32_t ch)   
{
//	ADC_ChannelConfTypeDef ADC_ChanConf;
//    
//	ADC_ChanConf.Channel=ch;                                   //ͨ��
//	ADC_ChanConf.Rank=ADC_REGULAR_RANK_1;                  	//1������
//	ADC_ChanConf.SamplingTime=ADC_SAMPLETIME_64CYCLES_5;      	//����ʱ��       
//	ADC_ChanConf.SingleDiff=ADC_SINGLE_ENDED;  				//���߲ɼ�          		
//	ADC_ChanConf.OffsetNumber=ADC_OFFSET_NONE;             	
//	ADC_ChanConf.Offset=0;   
//	HAL_ADC_ConfigChannel(hadc,&ADC_ChanConf);        //ͨ������

//	HAL_ADC_Start(hadc);                               //����ADC
//	
//	HAL_ADC_PollForConversion(hadc,10);                //��ѯת��
//	return (uint16_t)HAL_ADC_GetValue(hadc);	            //�������һ��ADC�������ת�����
		ADC_ChannelConfTypeDef ADC1_ChanConf;
    
    ADC1_ChanConf.Channel=ch;                                   //ͨ��
    ADC1_ChanConf.Rank=ADC_REGULAR_RANK_1;                  	//1������
    ADC1_ChanConf.SamplingTime=ADC_SAMPLETIME_64CYCLES_5;      	//����ʱ��       
		ADC1_ChanConf.SingleDiff=ADC_SINGLE_ENDED;  				//���߲ɼ�          		
		ADC1_ChanConf.OffsetNumber=ADC_OFFSET_NONE;             	
		ADC1_ChanConf.Offset=0;   
    HAL_ADC_ConfigChannel(&ADC1_Handler,&ADC1_ChanConf);        //ͨ������

    HAL_ADC_Start(&ADC1_Handler);                               //����ADC
	
    HAL_ADC_PollForConversion(&ADC1_Handler,10);                //��ѯת��
		return (u16)HAL_ADC_GetValue(&ADC1_Handler);	 
}

uint16_t Get_Adc_Average(ADC_HandleTypeDef *hadc,uint32_t ch,uint8_t times)
{
	uint32_t temp_val=0;
	uint8_t t;
	for(t=0;t<times;t++)
	{
//		temp_val+=Get_Adc(hadc,ch);
		temp_val+=Get_Adc(&ADC1_Handler,ch);
		//HAL_Delay(5);
	}
	return temp_val/times;
} 

void Read_All_Ad()
{
		adc1 = Get_Adc_Average(&ADC1_Handler,ADC_CHANNEL_16,AD_AVERAGE);
		adc2 = Get_Adc_Average(&ADC1_Handler,ADC_CHANNEL_17,AD_AVERAGE);
		adc3 = Get_Adc_Average(&ADC1_Handler,ADC_CHANNEL_14,AD_AVERAGE);
		adc4 = Get_Adc_Average(&ADC1_Handler,ADC_CHANNEL_15,AD_AVERAGE);
		adc5 = Get_Adc_Average(&ADC1_Handler,ADC_CHANNEL_18,AD_AVERAGE);
		adc_1_H = adc1>>8;
		adc_1_L = adc1&0xff;
		adc_2_H = adc2>>8;
		adc_2_L = adc2&0xff;
		adc_3_H = adc3>>8;
		adc_3_L = adc3&0xff;
		adc_4_H = adc4>>8;
		adc_4_L = adc4&0xff;
		adc_5_H = adc5>>8;
		adc_5_L = adc5&0xff;	
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hadc->Instance==ADC1)
  {
    __HAL_RCC_ADC12_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC1 GPIO Configuration    
		PA0			------> ADC1_INP16
    PA1     ------> ADC1_INP17
    PA2     ------> ADC1_INP14
    PA3     ------> ADC1_INP15
		PA4     ------> ADC1_INP18
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		
  }
			GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOD_CLK_ENABLE();					//����GPIOBʱ��
	
    GPIO_Initure.Pin=GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;			//PB0��1
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  		//�������
    GPIO_Initure.Pull=GPIO_NOPULL;         			//����
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;  	//����
    HAL_GPIO_Init(GPIOD,&GPIO_Initure);     		//��ʼ��GPIOB.0��GPIOB.1

    HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,GPIO_PIN_RESET);	//PB0��0
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,GPIO_PIN_RESET);	//PB1��1 
	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);	//PB0��0
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_RESET);	//PB1��1 

}
float remain_v;
void battery_indicatior(void)
{

	if(adc5>7.5*65536/4/3.3)
	{		
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,GPIO_PIN_RESET);	//PB0��0
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,GPIO_PIN_RESET);	//PB1��1 
	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);	//PB0��0
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_RESET);	//PB1��1 
	}
	if(adc5<=7.2*65536/4/3.3)
	{		
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,GPIO_PIN_SET);	//PB0��0
		if(adc5<6.85*65536/4/3.3)
		{				
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,GPIO_PIN_SET);	//PB1��1 
			if(adc5<6.6*65536/4/3.3)
			{					
				HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);	//PB0��0
				if(adc5<6.4*65536/4/3.3)
				{				
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_SET);	//PB1��1 
				}
			}
		}
	}
}