/*
 *######################################################################
 *                                RAppIDJDP
 *           Rapid Application Initialization and Documentation Tool
 *                         Freescale Semiconductor Inc.
 *
 *######################################################################
 *
 * Project Name           : TRK_MPC5606B_Example_DONGJU
 *
 * Project File           : TRK_MPC5606B_Example_DONGJU.rsp
 *
 * Revision Number        : 1.0
 *
 * Tool Version           : 1.2.1.5
 *
 * file                   : intc_pit.c
 *
 * Target Compiler        : Codewarrior
 *
 * Target Part            : MPC5606B
 *
 * Part Errata Fixes      : none
 *
 * Project Last Save Date : 06-Jul-2015 23:07:43
 *
 * Created on Date        : 06-Jul-2015 23:08:27
 *
 * Brief Description      : This  file contains  the interrupt service routine  for the Periodic Interrupt Timer
 *
 ******************************************************************************** 
 *
 * Detail Description     : This file is generated when PIT(Periodic Interrupt
 *                         Timer) function is defined in INTC peripheral.This
 *                         file contains the Interrupt handlers routines for PIT.
 *                         In Interrupt handlers routine respective flags are cleared.
 *
 ******************************************************************************** 
 *
 *######################################################################
*/

 
 
/********************  Dependent Include files here **********************/

#include "intc_pit.h"
#include "gpio_drv.h"

#define TIRE_LENGTH 16
#define ENCODER_LENGTH 14
#define TIRE_TO_CAMERA_LENGTH 24

uint32_t check123 = 0;
uint32_t count_Encoder =0;
uint32_t count_Pit1 = 0;
uint32_t speed123 = 0;
uint32_t wheelRotate = 0;

/************************* INTERRUPT HANDLERS ************************/

uint32_t Get_wheelRotate(void)
{
	return wheelRotate;
}

void Set_wheelRotate(uint32_t value)
{
	wheelRotate = value;
}


uint32_t Get_Speed(void)
{
	return speed123;
}

void PIT_CH0_ISR (void)
{
    PIT.CH[0].TFLG.R = 0x00000001;
}

void PIT_CH1_ISR (void) // 1/5000   0.0002초에 한번씩 인터럽트
{

	uint32_t tmp123;

    PIT.CH[1].TFLG.R = 0x00000001;
    tmp123 =GPIO_GetState(17);
    //tmp123이 1일때, 한바퀴 돈것인 듯 하다.
	//tmp123이 1이 되면, count_Encoder의 값을 1 증가.
	//그다음 tmp123이 0이 되면, 다시 1이 될때까지 기다림.
	//count_pit1의 값이 1000인 걸로 봐서 0.2초마다 속도를 갱신하는 듯 하다.
	
 	if(count_Pit1 ==0)
    {
       	speed123 = count_Encoder;   // 
       	count_Pit1 = 1000;
       	count_Encoder=0;
    }
 
   if(check123==0 &&  tmp123==1)
   {
	check123 =1;
	count_Encoder++;	
	wheelRotate++;
   }
   else if (check123==1 && tmp123==0)
   {
	check123 = 0;
   }
   count_Pit1 --;
  
}
 
/*
 *######################################################################
 *                           End of File
 *######################################################################
*/


