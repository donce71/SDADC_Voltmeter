/**
  ******************************************************************************
  * @file    SDADC/SDADC_Voltmeter/main.h 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    20-September-2012
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f37x.h"
#include "stm32373c_eval.h"
#include <stdio.h>
#include "string.h" // for memset
#include <stddef.h>
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/


/* A potentiometer (POT) is connected to PB1 which corresponds to SDADC1 channel 5P */
#define POT_GPIO_PORT        GPIOB
#define POT_GPIO_PIN         GPIO_Pin_1
#define POT_GPIO_CLK         RCC_AHBPeriph_GPIOB
#define POT_SDADC            SDADC1
#define POT_SDADC_CLK        RCC_APB2Periph_SDADC1
#define POT_SDADC_PWR        PWR_SDADCAnalog_1
#define POT_SDADC_VREF       SDADC_VREF_Ext /* External reference is selected */
#define POT_SDADC_GAIN       SDADC_Gain_1   /* Internal gain 1 is seleted: 
                                               SDADC_GAIN must be updated according to
                                               POT_SDADC_GAIN */
#define SDADC_GAIN           (float) 1  /* SDADC internal gain is set to 1: update this define*/
#define K_GAIN               (float) 1.03  /* Kalibravimo gain, cia pagal Vref_2.5V */


#define POT_SDADC_CHANNEL    SDADC_Channel_5
#define SDADC_RESOL          (int32_t) 65535 /* 2e16 - 1 */
#define SDADC_INIT_TIMEOUT   30 /* ~ about two SDADC clock cycles after INIT is set */
#define SDADC_CAL_TIMEOUT    4*30720 /*  ~5.12 ms at 6 MHz  in a single calibration sequence */
#define SDADC_VREF           (float) 3303  /* SDADC external reference is set to 3.3V */

#define RECEIVE                 1
#define TRANSMIT                0
#define N_DECIMAL_POINTS_PRECISION (1000) // n = 3. Three decimal points.

#define VREF_INTERNAL_BASE_ADDRESS  0x1FFFF7BA
#define ADC1_DR_Address             0x4001244C
#define SDADC1_DR_Address           0x40016060
#define Vref_ADRESS                 0x08004000
#define pol_ADRESS                  0x08004000+4
#define Tare_ADRESS                 0x08004000+8
#define COEFF_ADRESS                0x08004000+12


#define Vs_offsetPOL1          (float) 100 //Vsensor offset itampa kai POLiarumas 1 neigiamas, spaudziant dideja itampa
#define Vs_offsetPOL2          (float) 3100//Vsensor offset itampa kai POLiarumas 2 teigiamas, spaudziant mazeja itampa

// -----------------------     LIN    ------------------------------------//
/* Private define  */
#define bool _Bool
#define TRUE  1
#define FALSE 0
// frame id message define:
#define SENSOR_INFO			0x30
#define SENSOR_receive_OFFSET	            0x31
#define SENSOR_receive_SENSITIVITY		0x32
#define SENSOR_transmitt_OFFSET		0x33
#define SENSOR_transmitt_SENSITIVITY	0x34

// for USART init. RCC USART ijungimas ner define, surast ir pakeist kode.
#define LIN_USART       USART2
#define LIN_USART_IRQ   USART2_IRQn
#define LIN_IRQHandler	USART2_IRQHandler
/*
 *  Frame ID indicate about data length:
 *  0  - 31 '2 bytes'
 *  62 - 47	'4 bytes'
 *  48 - 63 '8 bytes'
 *  Bytes 60 - 63 are used for diagnostic frames.
 */
/* Private macro */
/* Private variables  for LIN commmunication*/
extern bool  LIN_frame_started ; // flag, frame starts after reak and sync byte.
extern uint8_t LIN_identifier; // six bits for frame identifier, value range 0 to 63 ; from this value depends data length.
extern uint8_t LIN_protected_identifier; // received frame ID with parity bits
extern bool LIN_header_received;
extern bool LIN_slave_is_subscriber; // by default device is publisher. If calibration command received switch to subscribe mode.
extern uint8_t LIN_RX_received_buffer[9], LIN_rx_cnt; // then slave act as subscriber
extern uint8_t LIN_transmission_active ;
extern uint8_t LIN_TX_buffer[9], LIN_tx_cnt, LIN_tx_total; // slave act as publisher
extern bool LIN_response_received ; // flag indicates then device get full frame response.
extern uint8_t Parse_value_switch; //for LIN subscriber values switch parsing
extern uint8_t debug_masyvas[10];
//Parse_value_switch = none;

 void USART_Send_data(uint8_t *buffer, uint32_t length);
/* Private function prototypes for application */
 void LIN_send_data (uint8_t *buffer, uint8_t length);
 void LIN_start_transmission(uint8_t *buffer, uint8_t length);
 uint8_t LIN_checksum_enhanced(uint8_t prot_ID, uint8_t *buffer, uint8_t length);
 uint8_t LIN_checksum_classic(uint8_t prot_ID, uint8_t *buffer, uint8_t length);
 uint8_t LIN_validate_response(uint8_t data_length);
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void TimingDelay_Decrement(void);
void Delay(__IO uint32_t nTime);
#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
