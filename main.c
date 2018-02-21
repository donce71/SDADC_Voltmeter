/**
  ******************************************************************************
  * @file    SDADC/SDADC_Voltmeter/main.c 
  * @author  Donatas
  * @version V1.23.9
  * @date    21-February-2018
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
  * 2018-02-02 SDADC skaitomas su DMA ir kalibravimo mechanizmas  
  * 2018-02-04 Pradedu rasyti koda plokstei su INA188 stiprintuvais
  * 2018-02-04 Naujas Post Office
  * 2018-02-08 PWMref target kompensacija (Vref) ir LIN
  * 2018-02-12 Newton perskaiciavimo funkcijos ir Flash irasymas koeficientu
  * 2018-02-19 Sitas kodas matuoja SDADC nuo 0mV, kopija Prezentacijos kodo, su Lin Be uzluzimo pakeitimais. Lin be uzluzimo neveikia gerai. Problema buvo su comment SDADC clock
  * 2018-02-21 Temperaturos matavimas pagal External 3V ref su SAR
******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdio.h"


/** @addtogroup SDADC_Voltmeter
  * @{
  */

/* Private macro -------------------------------------------------------------*/
GPIO_InitTypeDef        GPIO_InitStructure;
USART_InitTypeDef       USART_InitStructure;
NVIC_InitTypeDef        NVIC_InitStructure;
ADC_InitTypeDef         ADC_InitStructure;
GPIO_InitTypeDef        GPIO_InitStructure;
DMA_InitTypeDef         DMA_InitStructure;

/* Private variables ---------------------------------------------------------*/
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
float ADC_Vtemp=0; 
float ADC_Vref=0;
float ADC_3Vref_temp=0;
float new_temp=0;
float External_Vref=0;
float step_mv_new=0;
float step_mv=0;
float fixed_step=0;
float Vdd5V= 5000;
float Vdd5V_AVG= 5000;
float V_3v3_calculated=0;
float AVG_V_3v3_calculated=3300;
float Thermo_targetVpwm=0;
int16_t Newton=0;
int16_t Prior_Newton=0;
int16_t Newton_skirtumas=0;

/* Vidurkinimo variables */
uint16_t sumavimo_index1=0; 
uint16_t kaupimo_index1=0; 
float sumatorius1 = 0;
uint16_t sumavimo_index2=0; 
uint16_t kaupimo_index2=0; 
float sumatorius2 = 0;

/* PI controller variables */
float integral=0;
float integral2=0;
float integral3=0;

float targetVoltage=0;
float targetVref_mazas=0;
float AVG_error=0;
int8_t flag_send=0;
float Vdd5V_target=5000;

/*Flash memory vairables*/
uint32_t *temp;
int8_t offset_poliarumas;
float Newton_koef;
float zeroForce_mV;

/* Pagalbiniai variables */
uint32_t kintamasis=0;
float P_tu=2;
float I_tu=0;
uint8_t flag_calibruoti=0;
float RxBuffer[11]={0xFFF,0xFFF,0xFFF,0xFFF,0xFFF,0xFFF,0xFFF,0xFFF,0xFFF,0xFFF,0xFFF};
uint8_t calibravimo_rezult=0;
uint8_t flag_tare=0;
uint8_t flag_koef=0;
float naujas_Newton_koef=0;


/* Private variables  for LIN commmunication*/
bool  LIN_frame_started = 0; // flag, frame starts after reak and sync byte.
uint8_t LIN_identifier = 0; // six bits for frame identifier, value range 0 to 63 ; from this value depends data length.
uint8_t LIN_protected_identifier = 0; // received frame ID with parity bits
bool LIN_header_received = 0;
bool LIN_slave_is_subscriber = 0; // by default device is publisher. If calibration command received switch to subscribe mode.
uint8_t LIN_RX_received_buffer[9] = {0}, LIN_rx_cnt=0; // then slave act as subscriber
uint8_t LIN_transmission_active = 0;
uint8_t LIN_TX_buffer[9] = {0}, LIN_tx_cnt=0, LIN_tx_total=0; // slave act as publisher
uint8_t LIN_TX_data[9]={0};
bool LIN_response_received = 0; // flag indicates then device get full frame response.
uint8_t Parse_value_switch = SENSOR_receive_OFFSET; //for LIN subscriber values switch parsing
uint8_t debug_masyvas[10] = {0x10, 0x11, 0x12, 0x13, 0x14,0x15,0x16,0x17};
uint8_t debug = 0;
/* Private function prototypes -----------------------------------------------*/
float Get_8of10AVG(float buffer[]);
float termocompensation(float Vtemp);
float temperature(float ADC_vdd, float ADC_temperature);
float thermo_Vref(float temperatura_degree);
float Vrefpwm_thermo(float temp_deg, float Vrefpwm_target);
void Post_office( float *buffer_float);
float PI_controller(float value, float target, float Kp, float Ki);
float PI_con_5V(float value, float target, float Kp, float Ki);
uint16_t histereze_5V(float value, uint16_t current_PWM);
uint8_t offset_calib( void );
float PI_con_Vsensor(float value, float target, float Kp, float Ki);
void measureALL(void);
float sensor_init(void);
int16_t get_Newton(int8_t offset_pol, float sensor_mV, float zeroForce_mV, float keof);
void WriteValuesFLASH(float val1, uint8_t val2, float val3, float val4);
float SetTARE_and_set_Flash(void);


/* Periptheral function prototypes -------------------------------------------*/
static uint32_t SDADC1_Config(void);
static uint32_t SDADC3_Config(void);
void USART2_Configuration(void);
void GPIO_init(void);
void InitializeTimer(uint32_t PWM_PERIOD, uint32_t PWM_PERIOD_5V);
void InitializePWMChannel(void);
void ADC_init( void );
void DMA_initSDADC(void);
void ChangePWM_duty( uint16_t PULSE );
void ChangePWM_5V_duty( uint16_t PULSE );


int main(void)
{
  RCC_ClocksTypeDef RCC_Clocks;

  /* SysTick end of count event each 1ms */
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);
    
  GPIO_init();
  USART2_Configuration();
  InitializeTimer(PWM_PERIOD, PWM_PERIOD_5V);
  InitializePWMChannel();
  ADC_init();
  DMA_initSDADC();
 
  vref_internal_calibrated = *((uint16_t *)(VREF_INTERNAL_BASE_ADDRESS)); //ADC reiksme kai Vdd=3.3V
  Vref_internal_itampa= (vref_internal_calibrated)*3300/ 4095; //Vidinio Vref itampa
  Vref_internal_itampa= 1229;                // Kalibruojant su PICOLOG
  targetVoltage = sensor_init();

   //nuskaitymas is flash 
    temp = (uint32_t*)(pol_ADRESS);
  offset_poliarumas = *(uint8_t *)temp; 
    temp = (uint32_t*)(COEFF_ADRESS);
  Newton_koef = *(float *)temp; 
    temp = (uint32_t*)(Tare_ADRESS);
  zeroForce_mV = *(float *)temp; 
  
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
/*  KALIBRAVIMAS */      
      if (flag_calibruoti==1){
        calibravimo_rezult=offset_calib();//visi output daugiau nei 50decimal yra klaidos
        targetVoltage = sensor_init();
        //nuskaitymas is flash 
        temp = (uint32_t*)(pol_ADRESS);
        offset_poliarumas = *(uint8_t *)temp; 
        flag_calibruoti=0;
      }
      if (flag_tare==1){
        zeroForce_mV=SetTARE_and_set_Flash();
        flag_tare=0;
      }
      if (flag_koef==1){
        WriteValuesFLASH(targetVoltage, offset_poliarumas, zeroForce_mV, naujas_Newton_koef);
        Newton_koef=naujas_Newton_koef;
        flag_koef=0;
      }
      
/* ALL ADC AND SDADC MEASUREMENTS */
      measureALL();
      Thermo_targetVpwm=Vrefpwm_thermo( Tempe, targetVoltage);
            
/* Feedback: DUTY keiciu tik kas 100 matavimu, nes naudoju Vref AVG reiksme, kuri kinta tik cia if*/
      DUTY_5V = (uint16_t)PI_con_5V(Vdd5V, Vdd5V_target, 10,10);
      ChangePWM_5V_duty(DUTY_5V);
      DUTY=(uint16_t)PI_controller(VrefMv,Thermo_targetVpwm,3,10);   //30,0.2   0.6,0.1    10,20
      ChangePWM_duty( PWM_PERIOD - DUTY );

/* Convert to Newton */      
      Newton = get_Newton( offset_poliarumas, AVG_VsensorMv,  zeroForce_mV,  Newton_koef);
      Newton_skirtumas=Newton-Prior_Newton;

  //-----------------------------------   LIN pradzia   ----------------------------------------------------------
      LIN_TX_data[0]=(Newton & 0x00FF);      //lower  byte 1
      LIN_TX_data[1]=(Newton >> 8)& 0x00FF; //uper byte 1
      LIN_TX_data[2]=(Prior_Newton & 0x00FF); 
      LIN_TX_data[3]=(Prior_Newton >> 8)& 0x00FF; //uper  byte 2
      LIN_TX_data[4]=(Newton_skirtumas & 0x00FF); 
      LIN_TX_data[5]=(Newton_skirtumas >> 8)& 0x00FF; //uper  byte 3
      LIN_TX_data[6]=5;
      LIN_TX_data[7]=1;      
      
      if(LIN_header_received){ // slave received new command from master -> respond to message
          LIN_header_received = 0; // clear flag
          Parse_value_switch = 0; // clear value
          switch(LIN_identifier){
// Device will be publisher:
                case(SENSOR_INFO): // send sensor info
                          //sukrauti i masyva ir ...                                        
                          LIN_start_transmission(LIN_TX_data, 8);
                    break;
                case(SENSOR_calibration): //sensor calibration
                          //pradeti kalibravimo mechanizma
                          calibravimo_rezult=offset_calib();            //visi output daugiau nei 50decimal yra klaidos
                          targetVoltage = sensor_init();
                          //nuskaitymas is flash pakeistos reiksmes
                          temp = (uint32_t*)(pol_ADRESS);
                          offset_poliarumas = *(uint8_t *)temp; 
                          zeroForce_mV=SetTARE_and_set_Flash(); //taruoti butina po offset nuskaitymo
                    break;
                case(SENSOR_TARE): //sensor taravimas
                          zeroForce_mV=SetTARE_and_set_Flash();
                    break;
// Device will be subscriber:
                case(SENSOR_receive_OFFSET):  // sensor is receiving offset control value
                          LIN_slave_is_subscriber	= 1;
                          Parse_value_switch = SENSOR_receive_OFFSET;
                    break;
                case(SENSOR_receive_SENSITIVITY): // sensor is receiving sensitivity control value
                          LIN_slave_is_subscriber	= 1;
                          Parse_value_switch = SENSOR_receive_SENSITIVITY;
                    break;

// Device received command which is not valid
                  default:
                    break;
              }
            }

// LIN slave  is subscriber:
        if(LIN_response_received){
            LIN_response_received=0;
            switch(Parse_value_switch){
                case(SENSOR_receive_OFFSET):
// galima is LIN_RX_received_buffer[] pasiimti duomenis. 8 baitai [0 iki 7]. 8 yra checksum
//	            for(uint8_t i=0; i<9; i++)
//		debug_masyvas[i] = LIN_RX_received_buffer[i];
                  break;
                case(SENSOR_receive_SENSITIVITY):
                   //     naujas_Newton_koef  i sita sudet nauja reiksme;
                     naujas_Newton_koef =  Hex_to_float(LIN_RX_received_buffer, 1);
                    WriteValuesFLASH(targetVoltage, offset_poliarumas, zeroForce_mV, naujas_Newton_koef);
                    Newton_koef=naujas_Newton_koef;
                  break;
                  
                default: //error occured
                  break;
                }
                memset(LIN_RX_received_buffer, 0, 9); // clear buffer
        }
//------------------------------------------------------------------------------------------- LIN pabaiga  
    }//end of while loop
  }//end of else
}// end of main

int16_t get_Newton(int8_t offset_pol, float sensor_mV, float zeroForce_mV, float Newton_keof)
{ 
  int16_t Newton=0;
  float skirtumas_mV=0;
  
  switch (offset_pol){
    case 1: //spaudziant itampa dideja
      skirtumas_mV=sensor_mV-zeroForce_mV;
      break;
  case 2://spaudziant itampa mazeja
      skirtumas_mV=zeroForce_mV-sensor_mV;
    break;
      }
  
  Newton = (int16_t ) (skirtumas_mV/Newton_keof); 
  
  return Newton; 
}

float PI_controller(float value, float target, float Kp, float Ki)
{ 
  float output=0;
  float error=0;

  error=target-value;
  if ( ((DUTY<PWM_PERIOD) && (DUTY>0)) && (Ki!=0) ){
    integral = integral + (error)/8000;}       
  output= (Kp*error+Ki*integral);
  
      //apsauga nuo integral suoliu, RIBAS reikia parinkti pagal matavimo diapazona
  if ((DUTY==0 || DUTY==PWM_PERIOD) && (error>1500 || error<-1500)  )
      integral=0;
  
  if( output > PWM_PERIOD )
      output = PWM_PERIOD;
  if( output < 0 )
       output = 0;
    
  return output; 
}

float PI_con_Vsensor(float value, float target, float Kp, float Ki)
{ 
  
  float output=0;
  float error=0;

  error=target-value;
  if ( ((DUTY<PWM_PERIOD) && (DUTY>0)) && (Ki!=0) ){
    integral3 = integral3 + (error)/10;}       
  output= (Kp*error+Ki*integral3);
  
      //apsauga nuo integral suoliu, RIBAS reikia parinkti pagal matavimo diapazona
  if ((DUTY==0 || DUTY==PWM_PERIOD) && (error>1500 || error<-1500)  )
      integral=0;
     
  if( output > PWM_PERIOD )
      output = PWM_PERIOD;
  if( output < 0 )
       output = 0;
  
 /*       RxBuffer[0]=AVG_VrefMv;
        RxBuffer[1]=AVG_VsensorMv;
        RxBuffer[2]=DUTY;
        RxBuffer[3]=integral3;
        RxBuffer[4]=error;        
*/  
  return output; 
}

float PI_con_5V(float value, float target, float Kp, float Ki)
{ 
  float output=0;
  float error=0;
  
  error=target-value;
  if ( ((DUTY_5V<PWM_PERIOD_5V) && (DUTY_5V>0)) && (Ki!=0) ){
    integral2 = integral2 + (error)/100;} //reikia padauginti is iteration period______DEMESIO!!!!!!!!!!!!!!!!
  output= (Kp*error+Ki*integral2);
  
    //apsauga nuo integral suoliu, RIBAS reikia parinkti pagal matavimo diapazona
  if ((DUTY_5V==0 || DUTY_5V==PWM_PERIOD_5V) && (error>50 || error<-50)  )
      integral2=0;
  
  if( output > PWM_PERIOD_5V )
      output = PWM_PERIOD_5V;
  if( output < 0 )
       output = 0;
    
  return output; 
}

uint16_t histereze_5V(float value, uint16_t current_PWM) //pradinis variantas, dabar nenaudojama
{ 
  uint16_t output=0;
  
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
  
  return output; 
}

uint8_t offset_calib (void){
  
  uint8_t output=0;
  uint32_t i=0;
  uint8_t flag_baigta=0;
  uint8_t Poliarumas=0;
  float refPWM_calib=0;
  float temp_Vsensor=0;
  
     /* local variable definition */
  uint8_t state = 1;
  DUTY=0;

   while (flag_baigta==0){
     
   switch(state) {
      case 1 :
         GPIO_ResetBits(GPIOB, GPIO_Pin_4); // spaudziant dideja itampa: Multiplexer
         ChangePWM_duty( PWM_PERIOD - 0 );  //Vref = 2mV
         Delay(100); //ms
         measureALL();
         temp_Vsensor=((SDADCData_Tab[0] + 32768) * step_mv_new);
         if (temp_Vsensor <=100){
           state=2;}  // offset neigiamas
         else 
           state=3; //offset teigiamas
         break;
         
      case 2 : //algoritmas kai offset neigiamas
         // valdiklis su target 100mV, o matuojama itampa yra AVG_Vsensor
        for( i=1;i<7000;i++){                           //___________________3000 apie 10sekundziu
         measureALL();
         DUTY=(uint16_t)PI_con_Vsensor(VsensorMv, Vs_offsetPOL1, 0.5, 1.5); // target 50mV  0.2 1
         ChangePWM_duty( PWM_PERIOD - DUTY );
         Delay(2);
         //Post_office( RxBuffer);
        }
        if ( (AVG_VsensorMv>(Vs_offsetPOL1-2)) && (AVG_VsensorMv<(Vs_offsetPOL1+2))  ){
          output=1; // gerai sukalibruota neigiamas offset
          refPWM_calib= AVG_VrefMv+0.25;
          Poliarumas=1;
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
         temp_Vsensor=((SDADCData_Tab[0] + 32768) * step_mv_new);
         if ( temp_Vsensor >3180){
           state=4;}  // offset tikrai  teigiamas
         else {
           output=97; //klaida, ofset neveikia kaip teigiamas
           flag_baigta=1;}
         break;
         
      case 4 ://algoritmas kai offset teigiamas
         for( i=1;i<7000;i++){               //_________________________pamazinti laika
          measureALL();
          DUTY=(uint16_t)PI_con_Vsensor(VsensorMv, Vs_offsetPOL2, 0.2, 1); // target 3.18V  0.2 1
          ChangePWM_duty( PWM_PERIOD - DUTY ); 
          Delay(2);
          //Post_office( RxBuffer);
          }
         if (AVG_VsensorMv>(Vs_offsetPOL2-2) && AVG_VsensorMv<(Vs_offsetPOL2+2) ){
          output=4; // gerai sukalibruotas teigiamas offset
          refPWM_calib= AVG_VrefMv;
          Poliarumas=2;
          state=5;}
         else {
          output=95; // nepavyko sukalibruoti teigiamo offset  
          flag_baigta=1;}
         break;
         
       case 5 ://irasome i atminti
            WriteValuesFLASH(refPWM_calib, Poliarumas, zeroForce_mV,  Newton_koef);
            flag_baigta=1;
         break;

      default :
         output=99; //error
    }//switch
   }//while
  
  return output; 
}

void measureALL(void)
{
/* SAR ADC */
       ADC_Vref=RegularConvData_Tab[0]; 
       ADC_Vtemp=RegularConvData_Tab[1];
       ADC_3Vref_temp=RegularConvData_Tab[2];  
       
/* 3.6V arba 3.3V maitinimo itampos iskaiciavimas */ 
      fixed_step = 3000.45 /(SDADCData_Tab[1]+32768); // priimu kad External 3Vref cia nesikeicia (veliau ji kompensuosiu)
      V_3v3_calculated=(fixed_step*65535)+105.3; //1.037
      AVG_V_3v3_calculated = V_3v3_calculated + (V_3v3_calculated - AVG_V_3v3_calculated)/1000;     
       
/* Temperaturos iskaiciavimas */      
//       Vref_internal_itampa=termocompensation(ADC_Vtemp); 
//       VDD_ref=4095.0*(Vref_internal_itampa/ADC_Vref);
//       new_temp=temperature(VDD_ref,ADC_Vtemp);                 // atiduoda laipsnius
       new_temp=temperature(AVG_V_3v3_calculated,ADC_3Vref_temp);
       Tempe=Tempe+(new_temp-Tempe)/100;
       External_Vref = thermo_Vref(Tempe);
       
/* Compute the input voltage */   
      step_mv_new = External_Vref /(SDADCData_Tab[1]+32768);
      step_mv = step_mv + (step_mv_new - step_mv)/1000;
      
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
            if ( sumavimo_index1>99){
              AVG_VsensorMv= sumatorius1/100;  
              sumatorius1=0;
              sumavimo_index1=0; 
              Prior_Newton=Newton; // GAL TAIP DARYTI IR NEREIK
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
            if ( sumavimo_index2>99){
              AVG_VrefMv= sumatorius2/100;  
              sumatorius2=0;
              sumavimo_index2=0;
             }
        } 
}

void Post_office( float *buffer_float){
  uint8_t TxBufferIndex = 0;
  uint8_t temp[2];
  uint8_t j=0;
  uint8_t jj=0;
  int intIRdecimal[2];
  uint16_t buffer[50];
  float masyvas[11];
    for(uint8_t x=0; x<11; x++)
    {   masyvas[x]=*buffer_float;
        buffer_float++;
    }
      
  for (uint8_t x=0;x<50;x++)
      buffer[x]=0xFFF;

  while (masyvas[j]!=0xFFF ){
    
    intIRdecimal[0] = (int)masyvas[j];
    intIRdecimal[1] = ((int)(masyvas[j]*N_DECIMAL_POINTS_PRECISION)%N_DECIMAL_POINTS_PRECISION);
    
    for (jj=0; jj<2; jj++){
      temp[0] = intIRdecimal[jj] & 0xFF;
      temp[1] = (intIRdecimal[jj] >> 8) & 0xFF;
  
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
     }
    j=j+1;
  }
    
  while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    USART_SendData(USART2, 0xC0);
    
  TxBufferIndex=0;
  while(buffer[TxBufferIndex] != 0xFFF){
    while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    USART_SendData(USART2, buffer[TxBufferIndex]);
    TxBufferIndex++;
  }  
  while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    USART_SendData(USART2, 0xC1);
}


float termocompensation(float ADC_temperature){ //is ADC_temp gauna internal Vref itampa mV
    float result;
    //result= -0.00007*ADC_temperature*ADC_temperature + 0.1935*ADC_temperature + 1103.9;
    result= -0.008*ADC_temperature +1243;
    //ADC_Vref_komp=(float)Vref + (-0.0245)*(float)Vtemp + (0.000014 * (float)(Vtemp*Vtemp)); 
  return result;
}

float temperature(float ADC_VddmV, float ADC_temperature_bit){ //______________________Pataisyti
    float tempe_degree;
    float tempe_mV;
    tempe_mV=(ADC_VddmV/4095)*ADC_temperature_bit; 
    tempe_degree= 0.3788*tempe_mV -192.8 ;     
//    tempe_mV=(ADC_vdd/4095)*ADC_temperature; 
//    tempe_degree=- 0.24*tempe_mV +360 ; 
  return tempe_degree;
}

float thermo_Vref(float temperatura_degree){ //______________________Pataisyti
    float result1;
    result1 = -0.0025*temperatura_degree+3000.45;
  return result1;
}

float Vrefpwm_thermo(float temp_deg, float Vrefpwm_target){ 
  float result2;
  float norimas_Verfmazasis_prie_25C;
  float thermo_ratio;
  
//  thermo_ratio= temp_deg*temp_deg*(-0.000001)+temp_deg*0.00016+12.09;
  thermo_ratio=temp_deg*0.000148+12.0824;
  norimas_Verfmazasis_prie_25C= Vrefpwm_target/12.0869;
  
  if (temp_deg<30)
      result2=norimas_Verfmazasis_prie_25C*thermo_ratio;
  else
      result2=norimas_Verfmazasis_prie_25C*12.0869;
//  result2=norimas_Verfmazasis_prie_25C*thermo_ratio;
  return result2;
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
//=========================================================================================================================================================
/**
  * @brief  Sends array through LIN bus. Function have to wait till all bytes are send
  * @param  *buffer - pointer to array, length - data length 1...8 byte
  */
void LIN_send_data (uint8_t *buffer, uint8_t length)
{
	buffer[length] = LIN_checksum_enhanced(LIN_protected_identifier, buffer, length); //last byte in array must be checksum

	USART_Send_data(buffer, length+1);
}
/**
  * @brief  Sends array through LIN bus using USARTx interrupt. Transmission is  in background, no waiting
  * @param  *buffer - pointer to array, length - data length 1...8 byte
  */
void LIN_start_transmission(uint8_t *buffer, uint8_t length)
{
	uint8_t i=0;
       //     while(LIN_transmission_active) // palaukt kol pasibaigs sena
       //     {}
	LIN_tx_total = length+1; // 8 bytes and checksum
	LIN_transmission_active = 1;
	for(i=0; i<length; i++)
		LIN_TX_buffer[i] = buffer[i]; // put values in transmitt buffer.
	LIN_TX_buffer[length] = LIN_checksum_enhanced(LIN_protected_identifier, LIN_TX_buffer, length); //last byte in array must be checksum

	USART_ITConfig(LIN_USART, USART_IT_TXE, ENABLE);	//start transmitt, end of transmission is in interrupt handler;
}
// LIN spec 2.x use enhanced checksum. Frame ID is used.
uint8_t LIN_checksum_enhanced(uint8_t prot_ID, uint8_t *buffer, uint8_t length)
{
		uint8_t cheksum = 0;
		uint16_t temporal = 0;
		uint8_t i=0;

		temporal = prot_ID;                        
		for(i=0; i<length; i++){
			temporal +=buffer[i];
			if(temporal >= 256) // if owerflow, subtract 0xFF
                                      temporal = temporal - 255;
		}
		cheksum = 0xFF ^ (uint8_t)temporal; // invert
                      //  cheksum = 0xFF & (~temp);
		return cheksum;
}
// LIN spec 1.3 use classic checksum. Frame ID is not used, only data bytes.
uint8_t LIN_checksum_classic(uint8_t prot_ID, uint8_t *buffer, uint8_t length)
{
		uint8_t cheksum = 0;
		uint16_t temporal = 0;
		uint8_t i=0;

		for(i=0; i<length; i++){
			temp += buffer[i];
			if(temporal >= 256) // if owerflow, subtract 0xFF
				temporal = temporal - 255;
		}
		cheksum = 0xFF ^ temporal; // invert

		return cheksum;
}
/**
  * @brief  Clears flags after getting response to header
  * @param  length of received data bytes
  * @return 0 if response not valid, 1 if success.
  */
uint8_t LIN_validate_response(uint8_t data_length)
{
	uint8_t calc_checksum = 0;

	 LIN_rx_cnt = 0;
	 LIN_slave_is_subscriber = 0;
	 calc_checksum =  LIN_checksum_enhanced(LIN_protected_identifier, LIN_RX_received_buffer,data_length);
	 if(LIN_RX_received_buffer[data_length] == calc_checksum) // check if data is valid. last byte in array is checksum.
		 return 1;
	 else
		 return 0;
}
/**
  * @brief  Sends array through USART TX pin.
  * @param  *buffer - pointer to array, length - data length
  */
void USART_Send_data(uint8_t *buffer, uint32_t length)
{
		while(length--)
		{
			while (USART_GetFlagStatus(LIN_USART,USART_FLAG_TXE) == RESET)
			{}
			 USART_SendData(LIN_USART, *buffer);
			 buffer++;
		}
}
// grazina float reiksme padavus hex floatu masyva; start_index bus 1 pirmam skaiciui, 2 antram skaiciui
float Hex_to_float(uint8_t *array, uint8_t start_index)
{
  float *address = 0; // pointeris kad bus float
  uint8_t *p_array=0;
  uint8_t temp_array[8] = {0}, i=0;
  
  for(i=0; i<8; i++ )
    temp_array[i] = array[7-i]; // sudet reiksmes atvirksciatvarka i atminti;
  if(start_index==1)
    p_array = &temp_array[4];
  else if(start_index==2)
    p_array = &temp_array[0];
  else
    return 0;
  address = (float*)p_array;
    
  return *address;
}
//====================================================================================================================
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
  
  /*DMA_ITConfig(DMA2_Channel3, DMA_IT_TC, ENABLE);
  
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  */
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
  USART_InitTypeDef	USART_InitStructure;
  NVIC_InitTypeDef      NVIC_InitStructure;
  
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

  USART_InitStructure.USART_BaudRate = 19200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b ;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(LIN_USART, &USART_InitStructure);

  /* NVIC configuration */
  /* Configure the Priority Group to 2 bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = LIN_USART_IRQ ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  USART_ITConfig(LIN_USART, USART_IT_RXNE, ENABLE);
  USART_ITConfig(LIN_USART, USART_IT_TXE, DISABLE);
  USART_ITConfig(LIN_USART, USART_IT_ORE , ENABLE);
  USART_ITConfig(LIN_USART, USART_IT_LBD, ENABLE);
  
  USART_ITConfig(LIN_USART, USART_IT_FE, ENABLE);
  USART_ITConfig(LIN_USART, USART_IT_NE, ENABLE);

  USART_LINBreakDetectLengthConfig(LIN_USART, USART_LINBreakDetectLength_11b);
  USART_LINCmd(LIN_USART, ENABLE);
  /* Enable/disable the USART */
  USART_Cmd(LIN_USART, ENABLE);
}

void ADC_init( )
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
    /*Temperatura from 3V reference*/ 
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
  temp = (uint32_t*)Vref_ADRESS;
  output = *(float *)temp;
  temp = (uint32_t*)(pol_ADRESS);
  offset_pol = *(int8_t *)temp; // 1 - neigiamas offset, spaudziant dideja
                                // 2 - teigiamas offset, spaudziant mazeja

  if (offset_pol==1){
    GPIO_ResetBits(GPIOB, GPIO_Pin_4); // spaudziant dideja itampa: Multiplexer
  }
  else {
    GPIO_SetBits(GPIOB, GPIO_Pin_4);   // spaudziant mazeja itampa: Multiplexer 
  }
   return output;
}

float SetTARE_and_set_Flash(){
  uint16_t i=0;

   for( i=1;i<3000;i++){      
      Delay(1); 
/* ALL ADC AND SDADC MEASUREMENTS */
      measureALL();
      Thermo_targetVpwm=Vrefpwm_thermo( Tempe, targetVoltage);
/* Feedback: DUTY keiciu tik kas 100 matavimu, nes naudoju Vref AVG reiksme, kuri kinta tik cia if*/
      DUTY_5V = (uint16_t)PI_con_5V(Vdd5V, Vdd5V_target, 10,10);
      ChangePWM_5V_duty(DUTY_5V);
      DUTY=(uint16_t)PI_controller(VrefMv,Thermo_targetVpwm,5,10);   //30,0.2   0.6,0.1    10,20
      ChangePWM_duty( PWM_PERIOD - DUTY );
   }
  WriteValuesFLASH(targetVoltage, offset_poliarumas, AVG_VsensorMv, Newton_koef);
  
  return AVG_VsensorMv;
}


void WriteValuesFLASH(float val1, uint8_t val2, float val3, float val4){
  
  FLASH_Unlock();
  FLASH_ErasePage(Vref_ADRESS);
  FLASH_ProgramWord(Vref_ADRESS, *(uint32_t *)&val1); // Vref reiksme, kalibravimo metu
  FLASH_ProgramWord(pol_ADRESS, *(uint8_t *) &val2); // sensoriaus pajungimo poliskumas, 1-spaud, 2 - spaud mazeja
  FLASH_ProgramWord(Tare_ADRESS, *(uint32_t *)&val3); // Vs taravimo reiksme 
  FLASH_ProgramWord(COEFF_ADRESS, *(uint32_t *)&val4); // Newton konvertavimo koeficientas
  FLASH_Lock();
}

void ChangePWM_duty( uint16_t PULSE )
{
TIM4->CCR4=PULSE;
}

void ChangePWM_5V_duty( uint16_t PULSE )
{
TIM2->CCR1=PULSE;
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
