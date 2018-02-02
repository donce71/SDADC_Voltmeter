/**
  ******************************************************************************
  * @file    SDADC/SDADC_Voltmeter/main.c 
  * @author  Donatas
  * @version V1.12.6
  * @date    02-February-2018
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
  * 2018-01-16 Thermokompensations
  * 2018-01-23 Third SDADC for reference 
  * 2018-01-25 Modified SDADC
  * 2018-01-29 4th SDADC and PWM and 5V control
  * 2018-01-30 Histereze control
  * 2018-01-30 SAR ADC 5V control
  * 2018-01-31 External vref 3V, naujas perskaiciavimas
  * 2018-02-01 Tinkamas dreifas, Vsensor:~30mV, Vrefpwm: 0.2mV
  * 2018-02-02 SDADC skaitomas su DMA
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
int16_t InjectedConvDataCh7 = 0;
int16_t InjectedConvData3Ch7 = 0;

__IO uint32_t TimingDelay = 0;
uint32_t PWM_PERIOD=50000;
uint32_t PWM_PERIOD_5V=50000;
uint16_t DUTY=9424;
uint16_t DUTY_5V=16000;
__IO uint16_t RegularConvData_Tab[3];
__IO int16_t SDADCData_Tab[3];

/* ADC variables */
float VrefMv = 0;
float AVG_VrefMv=0;
float VsensorMv = 0;
float AVG_VsensorMv=0;
uint16_t vref_internal_calibrated = 0;
float Tempe=0;
float Vref_internal_itampa=0;
float VDD_ref=0;
float AVG_VDD_ref=3300;
float AVG_VDD_ref_NEW=3300;
float Voltage_buffer[10] = {0};
float Voltage_buffer2[10] = {0};
float Voltage_of_10=0;
float ADC_Vtemp=0; float ADC_Vref=0;
float new_temp=0;
float External_Vref=0;
float step_mv_new=0;
float step_mv=0;
float Vdd5V= 5000;
float Vdd5V_AVG= 5000;
float V_3v3_calculated=0;
float AVG_V_3v3_calculated=3300;
/* Flash write/read variables */


/* Vidurkinimo variables */
uint16_t sumavimo_index1=0; 
uint16_t kaupimo_index1=0; 
float sumatorius1 = 0;
uint16_t sumavimo_index2=0; 
uint16_t kaupimo_index2=0; 
float sumatorius2 = 0;

int integerPart;
int decimalPart;

/* PI controller variables */
float integral=0;
float integral2=0;
float integral3=0;

float targetVoltage=0;
float targetVref_mazas=0;
float AVG_error=0;
int8_t flag_send=0;
float Vdd5V_target=5000;

/* Pagalbiniai variables */
uint32_t kintamasis=0;
float P_tu=2;
float I_tu=0;
uint8_t flag_calibruoti=0;
/*


/* Private function prototypes -----------------------------------------------*/
static uint32_t SDADC1_Config(void);
static uint32_t SDADC3_Config(void);
void USART2_Configuration(void);
void RS485(uint8_t direction);
void GPIO_init(void);
void InitializeTimer(uint32_t PWM_PERIOD, uint32_t PWM_PERIOD_5V);
void InitializePWMChannel(void);
void ADC_init( void );
void DMA_initSDADC(void);
void ADC_measure (void);
float Get_8of10AVG(float buffer[]);
float termocompensation(float Vtemp);
float temperature(float ADC_vdd, float ADC_temperature);
float thermo_Vref(float temperatura_degree);

void Post_office( uint16_t v1, uint16_t v2, uint16_t v3);
void ChangePWM_duty( uint16_t PULSE );
void ChangePWM_5V_duty( uint16_t PULSE );
float PI_controller( float Kp, float Ki);
float PI_con_5V(float value, float target, float Kp, float Ki);
uint16_t histereze_5V(float value, uint16_t current_PWM);
uint8_t offset_calib( void );
float PI_con_Vsensor(float value, float target, float Kp, float Ki);
void measureALL(void);
float sensor_init(void);



/* Private functions ---------------------------------------------------------*/
uint32_t sadr;
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
  
//  sadr = (uint32_t)&SDADC1->JDATAR;
  
  GPIO_init();
  USART2_Configuration();
  InitializeTimer(PWM_PERIOD, PWM_PERIOD_5V);
  InitializePWMChannel();
  ADC_init();
  RS485(TRANSMIT);
  DMA_initSDADC();

 
  vref_internal_calibrated = *((uint16_t *)(VREF_INTERNAL_BASE_ADDRESS)); //ADC reiksme kai Vdd=3.3V
  Vref_internal_itampa= (vref_internal_calibrated)*3300/ 4095; //Vidinio Vref itampa
  Vref_internal_itampa= 1229;                // Kalibruojant su PICOLOG
  //targetVref_mazas=10;          
  //targetVoltage=targetVref_mazas*11.7;
  //targetVoltage=1800;
   targetVoltage = sensor_init();

 
  /* Test DMA1 TC flag */
  while((DMA_GetFlagStatus(DMA1_FLAG_TC1)) == RESET ); 
  /* Clear DMA TC flag */
  DMA_ClearFlag(DMA1_FLAG_TC1);

  /* SDADC1 */
  if(SDADC1_Config() != 0)
  {    while(1);     /* Forever loop */
  }
  /* SDADC3 */
  else if(SDADC3_Config() != 0)
  {    while(1);     /* Forever loop */
  }
  else
  {    
   while (1)
    {
      if (flag_calibruoti==1){
        if (offset_calib()>50){  //visi output daugiau nei 50 yra klaidos
          while(1);         } //Nepavyko sukalibtuoti
        flag_calibruoti=0;
      }

/* ALL ADC AND SDADC MEASUREMENTS */
        measureALL();
            
/* Feedback: DUTY keiciu tik kas 100 matavimu, nes naudoju Vref AVG reiksme, kuri kinta tik cia if*/
//    DUTY_5V = PI_con_5V(Vdd5V_AVG, Vdd5V_target, 10,3);
      DUTY_5V = histereze_5V(Vdd5V_AVG, DUTY_5V);
      ChangePWM_5V_duty(DUTY_5V);
      DUTY=PI_controller(30,0.2);   //0.6,0.1
      ChangePWM_duty( PWM_PERIOD - DUTY );
       
/* Transmit */
      if (flag_send==1){
        	/*Post_office( ADC_Vref,ADC_Vtemp,(VDD_ref-3000)*100); //paketas 1

	integerPart = (int)AVG_VrefMv;
	decimalPart = ((int)(AVG_VrefMv*N_DECIMAL_POINTS_PRECISION)%N_DECIMAL_POINTS_PRECISION);
	Post_office( integerPart,decimalPart,DUTY); //Paketas 2
 */           
            integerPart = (int)AVG_VsensorMv;
	decimalPart = ((int)(AVG_VsensorMv*N_DECIMAL_POINTS_PRECISION)%N_DECIMAL_POINTS_PRECISION);
	Post_office( integerPart,decimalPart,DUTY); //Paketas 3
      
            flag_send=0;
      }
      Delay(2); //_____________________________________DEMESIO
    }
  }
}

float PI_controller( float Kp, float Ki)
{ 
  float output=0;
  float error=0;
  float skirtumas=0;

  error=targetVoltage-VrefMv;
  //AVG_error=AVG_error+(AVG_error-error)/10;
  if ( ((DUTY>=PWM_PERIOD) || (DUTY<=0))!=1 ){
    integral = integral + (error);}       //reikia padauginti is iteration period______DEMESIO!!!!!!!!!!!!!!!!
  output= (Kp*error+Ki*integral)+25000;
  
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

float PI_con_Vsensor(float value, float target, float Kp, float Ki)
{ 
  float output=0;
  float error=0;
  float skirtumas=0;

  error=target-value;
  if ( ((DUTY>=PWM_PERIOD) || (DUTY<=0))!=1 ){
    integral3 = integral3 + (error);} 
  
  output= (Kp*error+Ki*integral3)+25000;
  
  if( output > PWM_PERIOD )
      output = PWM_PERIOD;
  if( output < 0 )
       output = 0;
  
  return output; 
}

float PI_con_5V(float value, float target, float Kp, float Ki)
{ 
  float output=0;
  float error=0;
  float skirtumas=0;

  error=target-value;
  if ( ((DUTY_5V>=PWM_PERIOD_5V) || (DUTY_5V<=0))!=1 ){
    integral2 = integral2 + (error* 4);} //reikia padauginti is iteration period______DEMESIO!!!!!!!!!!!!!!!!
  skirtumas=Ki*integral2;
  output= (Kp*error+Ki*integral2);
  
  if( output > PWM_PERIOD_5V )
      output = PWM_PERIOD_5V;
  if( output < 0 )
       output = 0;
  
  return output; 
}

uint16_t histereze_5V(float value, uint16_t current_PWM)
{ 
  uint16_t output=0;
  float error=0;
  float skirtumas=0;

  if (value> 5000.05){
    output=current_PWM-8;
  }
  else if (value< 4999.95){
    output=current_PWM+8;
  }
  else
    output=DUTY_5V;
  
  if( output > PWM_PERIOD_5V )
      output = PWM_PERIOD_5V;
  if( output < 0 )
       output = 0;
  
  return output; 
}

uint8_t offset_calib (void){
  
  uint8_t output=0;
  uint32_t i=0;
  uint32_t j=0;
  uint8_t flag_baigta=0;
  int8_t offset_poliarumas=0;
  float refPWM_target;
  
     /* local variable definition */
  uint8_t state = 1;

   while (flag_baigta==0){
     
   switch(state) {
      case 1 :
         GPIO_ResetBits(GPIOB, GPIO_Pin_4); // spaudziant dideja itampa: Multiplexer
         ChangePWM_duty( PWM_PERIOD - 0 );  //Vref = 2mV
         Delay(100); //ms
         measureALL();
         if (VsensorMv<=100)
           state=2;  // offset neigiamas
         else 
           state=3; //offset teigiamas
         break;
         
      case 2 : //algoritmas kai offset neigiamas
         // valdiklis su target 100mV, o matuojama itampa yra AVG_Vsensor
        for( i=1;i<15000;i++){
         measureALL();
         DUTY=PI_con_Vsensor(AVG_VsensorMv, 100, 0.003, 0.003);
         ChangePWM_duty( PWM_PERIOD - DUTY ); 
         
         integerPart = (int)AVG_VsensorMv;
         decimalPart = ((int)(AVG_VsensorMv*N_DECIMAL_POINTS_PRECISION)%N_DECIMAL_POINTS_PRECISION);
         Post_office( integerPart,decimalPart,DUTY); //Paketas 2
         
         Delay(2);
        }
        if (AVG_VsensorMv>98 && AVG_VsensorMv<102){
          output=1; // gerai sukalibruota neigiamas offset
          refPWM_target= AVG_VrefMv;
          offset_poliarumas=1;
          state=5; }
        else{
          output=98; // nepavyko sukalibruoti neigiamo offset
          state=3;}  //tegul bando su teigiamu
         break;
         
      case 3 : //algoritmas kai offset teigiamas, patikrinimas ar tikrai neigiamas
         GPIO_SetBits(GPIOB, GPIO_Pin_4); // spaudziant mazeja itampa: Multiplexer
         ChangePWM_duty( PWM_PERIOD - 50000 );  //Vref = 3300mV
         Delay(100); //ms
         measureALL();
         if (VsensorMv>3180)
           state=4;  // offset tikrai  teigiamas
         else {
           output=97; //klaida, ofset neveikia kaip teigiamas
           flag_baigta=1;}
         break;
         
      case 4 ://algoritmas kai offset teigiamas
         for( i=1;i<15000;i++){
          measureALL();
          DUTY=PI_con_Vsensor(AVG_VsensorMv, 3180, 0.01, 0.003); // target 3.18V
          ChangePWM_duty( PWM_PERIOD - DUTY ); 
         
         integerPart = (int)AVG_VsensorMv;
         decimalPart = ((int)(AVG_VsensorMv*N_DECIMAL_POINTS_PRECISION)%N_DECIMAL_POINTS_PRECISION);
         Post_office( integerPart,decimalPart,DUTY); //Paketas 2
         
          Delay(2);
          }
         if (AVG_VsensorMv>3178 && AVG_VsensorMv<3182){
          output=4; // gerai sukalibruotas teigiamas offset
          refPWM_target= AVG_VrefMv;
          offset_poliarumas=2;
          state=5;}
         else {
          output=95; // nepavyko sukalibruoti teigiamo offset  
          flag_baigta=1;}
         break;
         
       case 5 ://irasome i atminti
           FLASH_Unlock();
            FLASH_ErasePage(variable_ADRESS);
            FLASH_ProgramWord(variable_ADRESS, *(uint32_t *)&refPWM_target);
            FLASH_ProgramWord(variable_ADRESS+4, *(int8_t *)&offset_poliarumas);
            FLASH_Lock();
            flag_baigta=1;
         break;

      default :
         output=99; //error
   }
   }
  
  return output; 
}

void measureALL(void)
{
/* SAR ADC */
       ADC_Vref=RegularConvData_Tab[0]; 
       ADC_Vtemp=RegularConvData_Tab[1];
/* Thermocompensations */  
       
/*     ateiciai, maitinimo itampai tiksliai suzinoti ir nereikes tada vidinio ref naudoti  
      V_3v3_calculated=(step_mv*65535)*1.037;
      AVG_V_3v3_calculated = V_3v3_calculated + (V_3v3_calculated - AVG_V_3v3_calculated)/1000;
*/       
       Vref_internal_itampa=termocompensation(ADC_Vtemp); 
       VDD_ref=4095.0*(Vref_internal_itampa/ADC_Vref);
       new_temp=temperature(VDD_ref,ADC_Vtemp);                 // atiduoda laipsnius
       Tempe=Tempe+(new_temp-Tempe)/100;
       External_Vref = thermo_Vref(Tempe);
       //External_Vref=3000;
/* Compute the input voltage */   
      step_mv_new = External_Vref /(SDADCData_Tab[1]+32768);
      step_mv = step_mv + (step_mv_new - step_mv)/100;
      step_mv=step_mv_new;
      
      VsensorMv = (SDADCData_Tab[0] + 32768) * step_mv;
      VrefMv =    (SDADCData_Tab[2] + 32768) * step_mv;
      
      //5 Vdd matavimas
      Vdd5V =    ((InjectedConvData3Ch7 + 32768) * step_mv)/0.6175; //su Gwinstek matuojant 0.6175 atitnka 5V, kai varzos 2.4K ir 3.9K
      Vdd5V_AVG = Vdd5V_AVG + (Vdd5V - Vdd5V_AVG)/200;
      
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
             }
        } 
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
      /* SDADC1 channel 7P (PE9) */
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
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

    SDADC1->CR1 |= 1 << 16; // DMA channel enabled to read data from injected channel group

  
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
      /* select SDADC1 channel 7 to use conf0 */
  SDADC_ChannelConfig(SDADC1, SDADC_Channel_7, SDADC_Conf_0);

  /* select channel 4 8 7*/
  SDADC_InjectedChannelSelect(SDADC1, SDADC_Channel_4 | SDADC_Channel_8 | SDADC_Channel_7 );
  /* Enable continuous mode */
  SDADC_InjectedContinuousModeCmd(SDADC1, ENABLE);

  /* Exit initialization mode */
  SDADC_InitModeCmd(SDADC1, DISABLE);

  /* NVIC Configuration */
 /* NVIC_InitStructure.NVIC_IRQChannel = SDADC1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);*/

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
  SDADC_ITConfig(SDADC1, SDADC_IT_JEOC, DISABLE);
  Delay(500);
  /* Start a software start conversion */
  SDADC_SoftwareStartInjectedConv(SDADC1);
    
  return 0;
}

void DMA_initSDADC(void){
  
    /* DMA2 clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2 , ENABLE);
  
  /* DMA2 Channel1 Config */
  DMA_DeInit(DMA2_Channel3);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)SDADC1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)SDADCData_Tab;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 3;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA2_Channel3, &DMA_InitStructure);
  /* DMA2 Channel1 enable */
  DMA_Cmd(DMA2_Channel3, ENABLE);
  
  DMA_ITConfig(DMA2_Channel3, DMA_IT_TC, ENABLE);
  
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  

   
    /* Enable DMA */
  SDADC_DMAConfig(SDADC1, SDADC_DMATransfer_Regular, ENABLE);
  
}

static uint32_t SDADC3_Config(void)
{
  SDADC_AINStructTypeDef SDADC_AINStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  uint32_t SDADCTimeout = 0;

  /* SDADC1 APB2 interface clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SDADC3, ENABLE);
  
  /* PWR APB1 interface clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
  /* Enable SDADC1 analog interface */
  PWR_SDADCAnalogCmd(PWR_SDADCAnalog_3, ENABLE);

  /* Set the SDADC divider: The SDADC should run @6MHz */
  /* If Sysclk is 72MHz, SDADC divider should be 12 */
  RCC_SDADCCLKConfig(RCC_SDADCCLK_SYSCLK_Div48);

  /* RCC_AHBPeriph_GPIOB Peripheral clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

  /* SDADC1 channel 7P (PB15) */
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
    

  /* Select External reference: The reference voltage selection is available
     only in SDADC1 and therefore to select the VREF for SDADC2/SDADC3, SDADC1
     clock must be already enabled */
  SDADC_VREFSelect(SDADC_VREF_Ext);

  /* Insert delay equal to ~5 ms */
  Delay(5);
  
  /* Enable SDADC3 */
  SDADC_Cmd(SDADC3, ENABLE);
  
  /* Enter initialization mode */
  SDADC_InitModeCmd(SDADC3, ENABLE);
  SDADCTimeout = SDADC_INIT_TIMEOUT;
  /* wait for INITRDY flag to be set */
  while((SDADC_GetFlagStatus(SDADC3, SDADC_FLAG_INITRDY) == RESET) && (--SDADCTimeout != 0));

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
  SDADC_AINInit(SDADC3, SDADC_Conf_0, &SDADC_AINStructure);

  /* select SDADC3 channel 7 to use conf0 */
  SDADC_ChannelConfig(SDADC3, SDADC_Channel_7, SDADC_Conf_0);

  /* select channel 7*/
  SDADC_InjectedChannelSelect(SDADC3, SDADC_Channel_7 );
  /* Enable continuous mode */
  SDADC_InjectedContinuousModeCmd(SDADC3, ENABLE);

  /* Exit initialization mode */
  SDADC_InitModeCmd(SDADC3, DISABLE);

  /* NVIC Configuration */
  NVIC_InitStructure.NVIC_IRQChannel = SDADC3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* configure calibration to be performed on conf0 */
  SDADC_CalibrationSequenceConfig(SDADC3, SDADC_CalibrationSequence_1);
  /* start SDADC1 Calibration */
  SDADC_StartCalibration(SDADC3);
  /* Set calibration timeout: 5.12 ms at 6 MHz in a single calibration sequence */
  SDADCTimeout = SDADC_CAL_TIMEOUT;
  /* wait for SDADC1 Calibration process to end */
  while((SDADC_GetFlagStatus(SDADC3, SDADC_FLAG_EOCAL) == RESET) && (--SDADCTimeout != 0));
  
  if(SDADCTimeout == 0)
  {
    /* EOCAL flag can not set */
    return 2;
  }

  /* Enable end of injected conversion interrupt */
  SDADC_ITConfig(SDADC3, SDADC_IT_JEOC, ENABLE);
  /* Start a software start conversion */
  SDADC_SoftwareStartInjectedConv(SDADC3);
    
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
  DMA_InitStructure.DMA_BufferSize = 3;
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
  ADC_InitStructure.ADC_NbrOfChannel = 3;
  ADC_Init(ADC1, &ADC_InitStructure);
  
  /* Convert the ADC1 Channel 9 with 55.5 Cycles as sampling time */ 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 1, ADC_SampleTime_239Cycles5);
  /* Vtemperature */ 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, 2, ADC_SampleTime_239Cycles5);
    /* 5V Vdd*/ 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 3, ADC_SampleTime_239Cycles5);
  
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

float termocompensation(float ADC_temperature){ //is ADC_temp gauna internal Vref itampa mV
    float result;
    //result= -0.00007*ADC_temperature*ADC_temperature + 0.1935*ADC_temperature + 1103.9;
    result= -0.008*ADC_temperature +1243;
    //ADC_Vref_komp=(float)Vref + (-0.0245)*(float)Vtemp + (0.000014 * (float)(Vtemp*Vtemp)); 
  return result;
}

float temperature(float ADC_vdd, float ADC_temperature){ //______________________Pataisyti
    float tempe_degree;
    float tempe_mV;
    tempe_mV=(ADC_vdd/4095)*ADC_temperature; 
    tempe_degree=- 0.24*tempe_mV +360 ; 
    //tempe_degree=-0.0008*tempe_mV*tempe_mV + 1.7501*tempe_mV - 872.8 ; 
  return tempe_degree;
}

float thermo_Vref(float temperatura_degree){ //______________________Pataisyti
    float result;
      result = -0.0025*temperatura_degree+3000.45;
//    result = -0.012*temperatura_degree+3000.45; 
//    result = -0.0054*temperatura_degree+3000.45;   
//    result = temperatura_degree *0.067+2484.4;   
//    result = temperatura_degree *0.065+2484.4;
//    result = temperatura_degree*0.07+2577.5;
  return result;
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
  //PWM pin 5V
   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
 
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOA, &GPIO_InitStructure);   
   
  //Multiplexer pin
   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  
   
}

void InitializeTimer(uint32_t PWM_PERIOD, uint32_t PWM_PERIOD_5V)
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
    
    //Timer for 5V
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    timerInitStructure.TIM_Prescaler = 0;
    timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    timerInitStructure.TIM_Period = PWM_PERIOD_5V;
    timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    timerInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &timerInitStructure);
    TIM_Cmd(TIM2, ENABLE);
    
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

    //PWM for 5V
    TIM_OCInitTypeDef outputChannelInit2 = {1,};
    outputChannelInit2.TIM_OCMode = TIM_OCMode_PWM1;
    outputChannelInit2.TIM_Pulse = 0;
    outputChannelInit2.TIM_OutputState = TIM_OutputState_Enable;
    outputChannelInit2.TIM_OCPolarity = TIM_OCPolarity_High;
 
    TIM_OC1Init(TIM2, &outputChannelInit2);
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
 
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_1);    
}

float sensor_init(void)
{
uint32_t *temp;
int8_t offset_pol;
float output;

     //nuskaitymas is flash 
  temp = (uint32_t*)variable_ADRESS;
  output = *(float *)temp;
  temp = (uint32_t*)(variable_ADRESS+4);
  offset_pol = *(int8_t *)temp; // 1 - neigiamas offset, 2 - teigiamas offset

  if (offset_pol==1){
    GPIO_ResetBits(GPIOB, GPIO_Pin_4); // spaudziant dideja itampa: Multiplexer
  }
  else {
    GPIO_SetBits(GPIOB, GPIO_Pin_4);   // spaudziant mazeja itampa: Multiplexer 
  }
   return output;
}

void ChangePWM_duty( uint16_t PULSE )
{
TIM4->CCR4=PULSE;
}

void ChangePWM_5V_duty( uint16_t PULSE )
{
TIM2->CCR1=PULSE;
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
