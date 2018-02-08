/**
  ******************************************************************************
  * @file    SDADC/SDADC_Voltmeter/stm32f37x_it.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    20-September-2012
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
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

/* Includes ------------------------------------------------------------------*/
#include "stm32f37x_it.h"
#include "main.h"

/** @addtogroup STM32F37x_StdPeriph_Examples
  * @{
  */

/** @addtogroup SDADC_Voltmeter
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern int16_t InjectedConvDataCh4;
extern int16_t InjectedConvDataCh8;
extern int16_t InjectedConvDataCh7;
extern int16_t InjectedConvData3Ch7;
extern int16_t ping_pong = 0;



/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
 void  USART2_IRQHandler(void);
/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  TimingDelay_Decrement();
}

/******************************************************************************/
/*                 STM32F37x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f37x.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles SDADC1 interrupt request.
  * @param  None
  * @retval : None
  */
void SDADC1_IRQHandler(void)
{
  uint32_t ChannelIndex;
  
  if(SDADC_GetFlagStatus(SDADC1, SDADC_FLAG_JEOC) != RESET)
  {
    if (ping_pong==0){
    //Get the converted value
    InjectedConvDataCh4 = SDADC_GetInjectedConversionValue(SDADC1, &ChannelIndex);
    ping_pong+=1;
   }
    else if (ping_pong==1){
    InjectedConvDataCh7=SDADC_GetInjectedConversionValue(SDADC1, &ChannelIndex);
    ping_pong+=1;
    }
    else if (ping_pong==2){
    InjectedConvDataCh8=SDADC_GetInjectedConversionValue(SDADC1, &ChannelIndex);
    ping_pong=0;
    }
  } 
}

/**
  * @brief  This function handles SDADC3 interrupt request.
  * @param  None
  * @retval : None
  */
void SDADC3_IRQHandler(void)
{
  uint32_t ChannelIndex;
  
  if(SDADC_GetFlagStatus(SDADC3, SDADC_FLAG_JEOC) != RESET)
  {
    /* Get the converted value */
    InjectedConvData3Ch7 = SDADC_GetInjectedConversionValue(SDADC3, &ChannelIndex);  
  } 
}
/******************************************************************************/
 /*                LIN Interrupt Handlers                   */

 /******************************************************************************/
 void  USART2_IRQHandler(void)
 {
	 uint8_t new_data = 0; // most recently received byte in usart interrupt handler
	 uint8_t frame_parity = 0 ; //parity received
	 uint8_t parity_P0=0, parity_P1=0,  calculate_parity=0 ; // 2 parity bits calculated from equations;

   /* USART in mode Receiver --------------------------------------------------*/
   if (USART_GetITStatus(LIN_USART, USART_IT_RXNE) == SET)
   {
 	  	 USART_ClearITPendingBit(LIN_USART, USART_IT_RXNE);
 	  	 new_data = USART_ReceiveData(LIN_USART);

 	  	 //---- LIN device is subscriber
		  if(LIN_slave_is_subscriber){
			 LIN_RX_received_buffer[LIN_rx_cnt] = new_data;
			 LIN_rx_cnt++;
//			 if(LIN_rx_cnt ==9 ){ // 9 bytes received. Last byte is checksum.
//				 LIN_rx_cnt = 0;
//				 LIN_slave_is_subscriber = 0;
//				 calc_checksum =  LIN_checksum_enhanced(LIN_protected_identifier, LIN_RX_received_buffer,8);
//				 if(LIN_RX_received_buffer[8] == calc_checksum) // check if data is valid
//					 LIN_response_received = 1;
//			 }
			 //== Frame ID indicates data length.
			 if((LIN_identifier<32) && (LIN_rx_cnt ==3)){ // 2 data bytes and checksum
				 LIN_response_received = LIN_validate_response(2);
			 }

			 if((LIN_identifier<48) && (LIN_identifier>31) && (LIN_rx_cnt == 5)){// 4 data bytes and checksum
				 LIN_response_received = LIN_validate_response(4);
			 }
			 if((LIN_identifier<60) && (LIN_identifier>47) && (LIN_rx_cnt == 9)){// 8 data bytes and checksum
				 LIN_response_received = LIN_validate_response(8);
			 }
			 if(LIN_rx_cnt > 9) // for protection
				 LIN_rx_cnt = 0;
		 }
		  //---- LIN device is waiting for header start
 	  	 else if((new_data == 0x55) && (LIN_frame_started == 0))
 	  		LIN_frame_started = 1; // if lin frame started, next byte will be frame identifier

		  //---- LIN device is waiting for frame ID
 	  	 else if(LIN_frame_started){
 	  		 frame_parity = (new_data & 0xC0) >> 6; // parity received;
 	  		 parity_P0 = (new_data & 0x1) ^ ((new_data & 0x2)>>1) ^ ((new_data & 0x4)>>2) ^ ((new_data & 0x10)>>4) ; // P0 calculation: ID0 xor ID1 xor ID2 xor ID4
 	  		 parity_P1 = 1^(((new_data & 0x2)>>1) ^ ((new_data & 0x8)>>3) ^ ((new_data & 0x10)>>4) ^ ((new_data & 0x20)>>5));// P1 calculation: not(ID1 xor ID3 xor ID4 xor ID5)
 	  		 calculate_parity = parity_P0 | (parity_P1<<1); // parity calculated
 	  		 if( frame_parity ==  calculate_parity){ // if received parity is equal calculated -> ID received correctly, else ignore.
 	  		   	LIN_identifier = new_data & 0x3F; // 7, 8 bits are for parity!
 	  		   	LIN_protected_identifier = new_data;
				LIN_header_received = 1;
 	  		 }
 	  		 LIN_frame_started = 0; // clear flag.
 	  	 }

   }
	/* USART in mode transmitter  -------------------------------------------------------------------------------*/
	 if (USART_GetITStatus(LIN_USART, USART_IT_TXE) == SET)
	  {
	   USART_ClearITPendingBit(LIN_USART, USART_IT_TXE);
	   /* Write one byte to the transmit data register */
	  	   USART_SendData(LIN_USART,  LIN_TX_buffer[LIN_tx_cnt++]);
	      if(LIN_tx_cnt == LIN_tx_total)	// if all bytes are sent
	      {		// clear flags
	    	  LIN_transmission_active = 0;
	    	  memset(LIN_TX_buffer, 0, LIN_tx_cnt);
	    	  LIN_tx_cnt =0;
	    	  LIN_tx_total=0;
	        /* Disable the transmit interrupt */
	        USART_ITConfig(LIN_USART, USART_IT_TXE, DISABLE);
	  	  }
	  }
   /* USART detected line break -------------------------------------------------------------------------------*/
   if (USART_GetITStatus(LIN_USART, USART_IT_LBD) == SET)
    {
	   USART_ClearITPendingBit(LIN_USART, USART_IT_LBD);
	   USART_ClearITPendingBit(LIN_USART, USART_IT_FE);
	   USART_ClearITPendingBit(LIN_USART, USART_IT_ORE);
/* If break happens during communication = ABORT */
	   LIN_frame_started = 0;
	   LIN_identifier = 0;
	   LIN_protected_identifier = 0;
	   LIN_header_received = 0;
	   LIN_slave_is_subscriber = 0;
	   LIN_response_received = 0;
	   LIN_rx_cnt = 0;
	   memset(LIN_RX_received_buffer, 0, 9);
    }
   /* USART detected error -------------------------------------------------------------------------------*/
	  if (USART_GetITStatus(LIN_USART, USART_IT_ORE) == SET)
	   {
	   USART_ClearITPendingBit(LIN_USART, USART_IT_ORE);
	   }
 }
 /******************************************************************************/
/*
uint32_t cnt = 0;
void DMA2_Channel3_IRQHandler(void)
{
  if(DMA_GetITStatus(DMA2_IT_TC3) != RESET)
  {
    DMA_ClearITPendingBit(DMA2_IT_TC3);
    cnt++;
  }
}
*/
/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
