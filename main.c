/**
  ******************************************************************************
  * @file    SDADC/SDADC_Voltmeter/main.c 
  * @author  Donatas
  * @version V1.10.12
  * @date    12-January-2018
  * @brief   Main program body
  * Rx - PA2  TX - PA3, UART2 
  * SDADC PB2 
  * PWM TIM4ch4  PF6
  * feedback with Kp and Ki
  * papildomas ADC tikslumui
  * Vyto vidurkinmo metodas
  * Letas matavimas
  * siuncia po 4sps, svyruoja per 0.3mV, VDD +-20mV itakos matavimui neturi
  * trumpalaikis iki 50s matavimas telpa 0.1mV diapazone matuojant baterija
  * PWM period atskaitu 50k  0.066mV step   Periodas yra 1.28Khz , magistrale Timer yra 64Mhz
  * DMA
  * Termocompensation 
  * 2017-12-18 Second SDADC for signal
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdio.h"


/** @addtogroup SDADC_Voltmeter
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
GPIO_InitTypeDef    GPIO_InitStructure;
USART_InitTypeDef USART_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
ADC_InitTypeDef     ADC_InitStructure;
GPIO_InitTypeDef    GPIO_InitStructure;
DMA_InitTypeDef     DMA_InitStructure;


/* Private variables ---------------------------------------------------------*/
int16_t InjectedConvDataCh4 = 0;
int16_t InjectedConvDataCh8 = 0;
__IO uint32_t TimingDelay = 0;
uint32_t PWM_PERIOD=50000;
__IO uint16_t RegularConvData_Tab[2];

/* ADC variables */
float VrefMv = 0;
float AVG_VrefMv=0;
float VsensorMv = 0;
float AVG_VsensorMv=0;
__IO uint16_t  ADC1ConvertedValue[2] = {0,0};
uint16_t vref_internal_calibrated = 0;

float Vref_itampa=0;
float VDD_ref=0;
float AVG_VDD_ref=3300;
float Voltage_buffer[10] = {0};
float Voltage_buffer2[10] = {0};
float Voltage_of_10=0;
float ADC_Vtemp=0; float ADC_Vref=0;

/* Vidurkinimo variables */
uint16_t sumavimo_index1=0; 
uint16_t kaupimo_index1=0; 
float sumatorius1 = 0;
uint16_t sumavimo_index2=0; 
uint16_t kaupimo_index2=0; 
float sumatorius2 = 0;

int integerPart;
int decimalPart;
uint16_t DUTY=9424;
/* PI controller variables */
float Kp=0.6;
float Ki=0.1;
float integral=0;
float targetVoltage=0;
float targetVref_mazas=0;
float AVG_error=0;
int8_t flag_send=0;

/* Pagalbiniai variables */
uint32_t kintamasis=0;



/* Private function prototypes -----------------------------------------------*/
static uint32_t SDADC1_Config(void);
void USART2_Configuration(void);
void RS485(uint8_t direction);
void GPIO_init(void);
void InitializeTimer(uint32_t PWM_PERIOD);
void InitializePWMChannel(void);
void ADC_init( void );
void ADC_measure (void);
float Get_8of10AVG(float buffer[]);
float termocompensation(float Vtemp, float Vref);



void Post_office( uint16_t v1, uint16_t v2, uint16_t v3);
void ChangePWM_duty( uint16_t PULSE );
float PI_controller(void);
float V_target_computation(float vref, float Vdd3V3);


/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  
  RCC_ClocksTypeDef RCC_Clocks;

  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f37x.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f37x.c file
     */
  /* SysTick end of count event each 1ms */
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);
  
  GPIO_init();
  USART2_Configuration();
  InitializeTimer(PWM_PERIOD);
  InitializePWMChannel();
  ADC_init();
  RS485(TRANSMIT);
    
  vref_internal_calibrated = *((uint16_t *)(VREF_INTERNAL_BASE_ADDRESS)); //ADC reiksme kai Vdd=3.3V
  Vref_itampa= (vref_internal_calibrated)*3300/ 4095; //Vidinio Vref itampa
  Vref_itampa= 1229;                // Kalibruojant su PICOLOG
  targetVref_mazas=150;          
  targetVoltage=targetVref_mazas*17.3;
  //targetVoltage=620;

  
  ChangePWM_duty( PWM_PERIOD/2 );
    
  /* Test DMA1 TC flag */
  while((DMA_GetFlagStatus(DMA1_FLAG_TC1)) == RESET ); 
    
  /* Clear DMA TC flag */
  DMA_ClearFlag(DMA1_FLAG_TC1);

  /* SDADC1 */
  if(SDADC1_Config() != 0)
  {    while(1);     /* Forever loop */
  }
  else
  {
/* INIT values*/
   ADC_Vref =RegularConvData_Tab[0]; 
   ADC_Vtemp=RegularConvData_Tab[1]; 
    
   while (1)
    {
/* SAR ADC with thermocompensation*/
       ADC_Vref=RegularConvData_Tab[0]; 
       ADC_Vtemp=RegularConvData_Tab[1];
       VDD_ref=(4095.0*Vref_itampa)/termocompensation(ADC_Vtemp, ADC_Vref);
       AVG_VDD_ref=VDD_ref; 
       
/* Compute the input voltage */       
      VsensorMv = (((InjectedConvDataCh4 + 32768) * AVG_VDD_ref) / (SDADC_GAIN *K_GAIN * SDADC_RESOL));
      VrefMv = (((InjectedConvDataCh8 + 32768) * AVG_VDD_ref) / (SDADC_GAIN *K_GAIN * SDADC_RESOL));
      
/* vidurkinimas Vsensor*/
       if (kaupimo_index1<10){     //kaupiame 10 reiksmiu bufferi
          Voltage_buffer[kaupimo_index1]=VsensorMv;
          kaupimo_index1++;
          }
       else{                     // apskaiciuoja buferio vidurki is 8 reiksmiu (be min ir max)
            Voltage_of_10=Get_8of10AVG(Voltage_buffer);
            kaupimo_index1=0;  
            sumatorius1+=Voltage_of_10;
            sumavimo_index1++;
            if ( sumavimo_index1>9){
              AVG_VsensorMv= sumatorius1/10;  
              sumatorius1=0;
              sumavimo_index1=0; 
            }
        }
       
/* vidurkinimas Vref*/
       if (kaupimo_index2<10){     //kaupiame 10 reiksmiu bufferi
          Voltage_buffer2[kaupimo_index2]=VrefMv;
          kaupimo_index2++;
          }
       else{                     // apskaiciuoja buferio vidurki is 8 reiksmiu (be min ir max)
            Voltage_of_10=Get_8of10AVG(Voltage_buffer2);
            kaupimo_index2=0;  
            sumatorius2+=Voltage_of_10;
            sumavimo_index2++;
            if ( sumavimo_index2>9){
              AVG_VrefMv= sumatorius2/10;  
              sumatorius2=0;
              sumavimo_index2=0;
	  flag_send=1;
/* Feedback: DUTY keiciu tik kas 100 matavimu, nes naudoju Vref AVG reiksme, kuri kinta tik cia if*/
              DUTY=PI_controller();
              ChangePWM_duty( PWM_PERIOD - DUTY );
             }
        }
       
/* Transmit */
      if (flag_send==1){
        	Post_office( ADC_Vref,ADC_Vtemp,(VDD_ref-3000)*100); //paketas 1

	integerPart = (int)AVG_VrefMv;
	decimalPart = ((int)(AVG_VrefMv*N_DECIMAL_POINTS_PRECISION)%N_DECIMAL_POINTS_PRECISION);
	Post_office( integerPart,decimalPart,0); //Paketas 2
            
            integerPart = (int)AVG_VsensorMv;
	decimalPart = ((int)(AVG_VsensorMv*N_DECIMAL_POINTS_PRECISION)%N_DECIMAL_POINTS_PRECISION);
	Post_office( integerPart,decimalPart,DUTY); //Paketas 3
            
            /*integerPart = (int)targetVoltage;
	decimalPart = ((int)(targetVoltage*N_DECIMAL_POINTS_PRECISION)%N_DECIMAL_POINTS_PRECISION);           
            Post_office( integerPart,decimalPart,0); //paketas 4
            */
            flag_send=0;
      }
      Delay(2); //_____________________________________DEMESIO
    }
  }
}

float PI_controller()
{ 
  float output=0;
  float error=0;
  float skirtumas=0;

  
  error=targetVoltage-AVG_VrefMv;
  AVG_error=AVG_error+(AVG_error-error)/10;
  integral = integral + (error* 4); //reikia padauginti is iteration period______DEMESIO!!!!!!!!!!!!!!!!
  output= (Kp*error+Ki*integral);
  
  if( output > PWM_PERIOD )
      output = PWM_PERIOD;
  if( output < 0 )
       output = 0;
  
  /*skirtumas=output-(float)DUTY;
  if ( (skirtumas<1) && (skirtumas>-1) ){
    output=DUTY;
  }*/

  return output; 
}

float V_target_computation(float vref, float Vdd3V3) //vref norimas mazasis vref, VISI SKAICIAVIAMAI VOLTAIS
{ 
  float Vddminus=0;
  float Vpwm=0;
  float r0=240;
  float r1=1200;
  float r2=2400;
  float t1=0;
  float t2=0;
  float t3=0;

  //Apskaiciuoju minusine VDD
  Vddminus=(Vdd3V3*-2.3944+4628.7)/1000;
  
  //Apskaicuoju Vref didiji target
  /*t1= ( vref*r0-Vddminus*r0  ) * ( (r1*r2/r0) +r1+r2 )  ;
  t2=(r0*r2)  - (  r1*(Vddminus)/r0  ) + Vddminus;
  t3=  ( vref*r0-Vddminus*r0  ) * ( (r1*r2/r0) +r1+r2 )    /  (r0*r2)  ;
  */
  Vpwm= (  ( ( vref*r0-Vddminus*r0  ) * ( (r1*r2/r0) +r1+r2 )  )  /  (r0*r2)  ) - (  r1*(-Vddminus)/r0  ) + Vddminus;

  return (Vpwm*1000); 
}

void Post_office( uint16_t v1, uint16_t v2, uint16_t v3){
  uint8_t TxBufferIndex = 0;
  uint8_t temp[2];
  uint16_t buffer[12] = {0x0FFF,0x0FFF, 0x0FFF,0x0FFF, 0x0FFF, 0x0FFF, 0x0FFF, 0x0FFF, 0x0FFF,0x0FFF, 0x0FFF, 0x0FFF};
    
  //Maskavimas  V1
  temp[0] = v1 & 0xFF;
  temp[1] = (v1 >> 8) & 0xFF;
  
  if((temp[0] == 0xFE) || (temp[0] == 0xC0) || (temp[0] == 0xC1)){
    buffer[TxBufferIndex++] = 0xFE;
    buffer[TxBufferIndex++] = temp[0];
  }else{
    buffer[TxBufferIndex++] = temp[0];  }
  if((temp[1] == 0xFE) || (temp[1] == 0xC0) || (temp[1] == 0xC1)){
    buffer[TxBufferIndex++] = 0xFE;
    buffer[TxBufferIndex++] = temp[1];
  }else{
    buffer[TxBufferIndex++] = temp[1];  }
    //Maskavimas  V2
  temp[0] = v2 & 0xFF;
  temp[1] = (v2 >> 8) & 0xFF;
  
  if((temp[0] == 0xFE) || (temp[0] == 0xC0) || (temp[0] == 0xC1)){
    buffer[TxBufferIndex++] = 0xFE;
    buffer[TxBufferIndex++] = temp[0];
  }else{
    buffer[TxBufferIndex++] = temp[0];  }
  if((temp[1] == 0xFE) || (temp[1] == 0xC0) || (temp[1] == 0xC1)){
    buffer[TxBufferIndex++] = 0xFE;
    buffer[TxBufferIndex++] = temp[1];
  }else{
    buffer[TxBufferIndex++] = temp[1];  }
    //Maskavimas  V3
  temp[0] = v3 & 0xFF;
  temp[1] = (v3 >> 8) & 0xFF;
  
  if((temp[0] == 0xFE) || (temp[0] == 0xC0) || (temp[0] == 0xC1)){
    buffer[TxBufferIndex++] = 0xFE;
    buffer[TxBufferIndex++] = temp[0];
  }else{
    buffer[TxBufferIndex++] = temp[0];  }
  if((temp[1] == 0xFE) || (temp[1] == 0xC0) || (temp[1] == 0xC1)){
    buffer[TxBufferIndex++] = 0xFE;
    buffer[TxBufferIndex++] = temp[1];
  }else{
    buffer[TxBufferIndex++] = temp[1];  }
  //_________end_maskavimas
  
  
  while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    USART_SendData(USART2, 0xC0);
    
  TxBufferIndex=0;
  while(buffer[TxBufferIndex] != 0xFFF){
    while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    USART_SendData(USART2, buffer[TxBufferIndex]);
    TxBufferIndex++;
  }  
}


/**
  * @brief  Configure SDADC1 channel 4P in Single Ended Zero Reference mode using
  *         injected conversion with continuous mode enabled.
  * @param  None
  * @retval 0: SDADC Configured successfully
  *         1: INITRDY flag is not set, check the SDVDDA and SDVREF
  *         2: EOCAL flag is not set
  */
static uint32_t SDADC1_Config(void)
{
  SDADC_AINStructTypeDef SDADC_AINStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  uint32_t SDADCTimeout = 0;

  /* SDADC1 APB2 interface clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SDADC1, ENABLE);
  
  /* PWR APB1 interface clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
  /* Enable SDADC1 analog interface */
  PWR_SDADCAnalogCmd(PWR_SDADCAnalog_1, ENABLE);

  /* Set the SDADC divider: The SDADC should run @6MHz */
  /* If Sysclk is 72MHz, SDADC divider should be 12 */
  RCC_SDADCCLKConfig(RCC_SDADCCLK_SYSCLK_Div48);

  /* RCC_AHBPeriph_GPIOB Peripheral clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

  /* SDADC1 channel 4P (PB2) */
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
    /* RCC_AHBPeriph_GPIOE Peripheral clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE, ENABLE);
  
    /* SDADC1 channel 8P (PE8) */
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  

  /* Select External reference: The reference voltage selection is available
     only in SDADC1 and therefore to select the VREF for SDADC2/SDADC3, SDADC1
     clock must be already enabled */
  SDADC_VREFSelect(SDADC_VREF_Ext);

  /* Insert delay equal to ~5 ms */
  Delay(5);
  
  /* Enable SDADC1 */
  SDADC_Cmd(SDADC1, ENABLE);
  
  /* Enter initialization mode */
  SDADC_InitModeCmd(SDADC1, ENABLE);
  SDADCTimeout = SDADC_INIT_TIMEOUT;
  /* wait for INITRDY flag to be set */
  while((SDADC_GetFlagStatus(SDADC1, SDADC_FLAG_INITRDY) == RESET) && (--SDADCTimeout != 0));

  if(SDADCTimeout == 0)
  {
    /* INITRDY flag can not set */
    return 1;
  }

  /* Analog Input configuration conf0: use single ended zero reference */
  SDADC_AINStructure.SDADC_InputMode = SDADC_InputMode_SEZeroReference;
  SDADC_AINStructure.SDADC_Gain = SDADC_Gain_1;
  SDADC_AINStructure.SDADC_CommonMode = SDADC_CommonMode_VSSA;
  SDADC_AINStructure.SDADC_Offset = 0;
  SDADC_AINInit(SDADC1, SDADC_Conf_0, &SDADC_AINStructure);

  /* select SDADC1 channel 4 to use conf0 */
  SDADC_ChannelConfig(SDADC1, SDADC_Channel_4, SDADC_Conf_0);
    /* select SDADC1 channel 8 to use conf0 */
  SDADC_ChannelConfig(SDADC1, SDADC_Channel_8, SDADC_Conf_0);

  /* select channel 4 */
  SDADC_InjectedChannelSelect(SDADC1, SDADC_Channel_4 | SDADC_Channel_8);
    /* select channel 8 */
  /* Enable continuous mode */
  SDADC_InjectedContinuousModeCmd(SDADC1, ENABLE);

  /* Exit initialization mode */
  SDADC_InitModeCmd(SDADC1, DISABLE);

  /* NVIC Configuration */
  NVIC_InitStructure.NVIC_IRQChannel = SDADC1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* configure calibration to be performed on conf0 */
  SDADC_CalibrationSequenceConfig(SDADC1, SDADC_CalibrationSequence_1);
  /* start SDADC1 Calibration */
  SDADC_StartCalibration(SDADC1);
  /* Set calibration timeout: 5.12 ms at 6 MHz in a single calibration sequence */
  SDADCTimeout = SDADC_CAL_TIMEOUT;
  /* wait for SDADC1 Calibration process to end */
  while((SDADC_GetFlagStatus(SDADC1, SDADC_FLAG_EOCAL) == RESET) && (--SDADCTimeout != 0));
  
  if(SDADCTimeout == 0)
  {
    /* EOCAL flag can not set */
    return 2;
  }

  /* Enable end of injected conversion interrupt */
  SDADC_ITConfig(SDADC1, SDADC_IT_JEOC, ENABLE);
  /* Start a software start conversion */
  SDADC_SoftwareStartInjectedConv(SDADC1);
    
  return 0;
}

void USART2_Configuration(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);


  /* Configure USART1 RX and Tx*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
   /* Connect to pins */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_7);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_7);

  /* Configure USART1 */
  USART_InitStructure.USART_BaudRate = 115200; // Baud rate. Bits (or characters) per second.
  USART_InitStructure.USART_WordLength = USART_WordLength_8b; // Word lenght = 8 bits
  USART_InitStructure.USART_StopBits = USART_StopBits_1; // Use one stop bit
  USART_InitStructure.USART_Parity = USART_Parity_No ; // No parity
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // Receive and transmit enabletd
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // Hardware flow control disabled (RTS and CTS signals)
  USART_Init(USART2, &USART_InitStructure);
  
  USART_Cmd(USART2, ENABLE); // Enable USART1 parameters
  
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // Enable Rx interrupt
 
  //NVIC_EnableIRQ(USART1_IRQn); // Enable USART1 global interrupt
  
  /* NVIC structure for USART1 interrupt */
  /*NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);*/
}

void ADC_init(  )
{ 
 
  /* ADCCLK = PCLK2/4 */
  RCC_ADCCLKConfig(RCC_PCLK2_Div4); 
  
  /* DMA1 clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , ENABLE);
  
  /* DMA1 Channel1 Config */
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)RegularConvData_Tab;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 2;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  /* DMA1 Channel1 enable */
  DMA_Cmd(DMA1_Channel1, ENABLE);
  
  /* ADC1 Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  
  /* ADCs DeInit */  
  ADC_DeInit(ADC1);
  
    /* Enable ADC_DMA */
  ADC_DMACmd(ADC1, ENABLE);  
  
  /* Initialize ADC structure */
  ADC_StructInit(&ADC_InitStructure);
  
  /* Configure the ADC1 in continuous mode */
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 2;
  ADC_Init(ADC1, &ADC_InitStructure);
  
  /* Convert the ADC1 Channel 9 with 55.5 Cycles as sampling time */ 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 1, ADC_SampleTime_239Cycles5);
  /* Vtemperature */ 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, 2, ADC_SampleTime_239Cycles5);
  
  ADC_TempSensorVrefintCmd(ENABLE);
    
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);  
  
  /* ADC1 reset calibration register */   
  ADC_ResetCalibration(ADC1);
  
  while(ADC_GetResetCalibrationStatus(ADC1));
  
  /* ADC1 calibration start */
  ADC_StartCalibration(ADC1);
    
  while(ADC_GetCalibrationStatus(ADC1)); 
  
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);     
}

float termocompensation(float Vtemp, float Vref){ //grazina ADC_Vref_kompensuotas
    float ADC_Vref_komp;
    //ADC_Vref_komp=(float)Vref + (-0.0245)*(float)Vtemp + (0.000014 * (float)(Vtemp*Vtemp)); 
     ADC_Vref_komp=(float)Vref*0.995 + (-0.003)*(float)Vtemp + (0.000005 * (float)(Vtemp*Vtemp)); 
  return ADC_Vref_komp;
}

float Get_8of10AVG(float buffer[] )
{
  float average = 0;
  float min = buffer[0];
  uint8_t minIND=0;
  float max = buffer[9];
  uint8_t maxIND=9;


  for(uint8_t i = 0; i < 10; i++)
  {
    if (buffer[i]<min){
      min=buffer[i];
      minIND=i;
    }
    if (buffer[i]>max){
      max=buffer[i];
      maxIND=i;
    }
  }
  if (minIND==maxIND)
    maxIND=minIND+1;
  buffer[minIND]=0;
  buffer[maxIND]=0;
  
  for(uint8_t i = 0; i < 10; i++)
    average +=buffer[i];
  
    average /= 8;

  return average; 
}

void GPIO_init(){
  
  GPIO_InitTypeDef GPIO_InitStructure;
  
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
      
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; /*Configure RS485*/
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  //PWM pin
   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);
 
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOF, &GPIO_InitStructure);
}

void InitializeTimer(uint32_t PWM_PERIOD)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    TIM_TimeBaseInitTypeDef timerInitStructure;
    timerInitStructure.TIM_Prescaler = 0;
    timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    timerInitStructure.TIM_Period = PWM_PERIOD;
    timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    timerInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &timerInitStructure);
    TIM_Cmd(TIM4, ENABLE);
}

void InitializePWMChannel()
{
    TIM_OCInitTypeDef outputChannelInit = {4,};
    outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
    outputChannelInit.TIM_Pulse = 0;
    outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
    outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;
 
    TIM_OC4Init(TIM4, &outputChannelInit);
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
 
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource6, GPIO_AF_2);
}

void ChangePWM_duty( uint16_t PULSE )
{
TIM4->CCR4=PULSE;
}

void RS485(uint8_t direction){
  if(direction == TRANSMIT){
    GPIO_SetBits(GPIOA, GPIO_Pin_4);
  }else if(direction == RECEIVE){
    GPIO_ResetBits(GPIOA, GPIO_Pin_4);
  }
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in 1 ms.
  * @retval None
  */
void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
