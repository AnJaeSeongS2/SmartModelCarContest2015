/*
 *######################################################################
 *                                RAppIDJDP
 *           Rapid Application Initialization and Documentation Tool
 *                         Freescale Semiconductor Inc.
 *
 *######################################################################
 *
 * Project Name           : wing_rsp_file_vol_1
 *
 * Project File           : wing_rsp_file_vol_1.rsp
 *
 * Revision Number        : 1.0
 *
 * Tool Version           : 1.2.1.5
 *
 * file                   : intc_pit.h
 *
 * Target Compiler        : Codewarrior
 *
 * Target Part            : MPC5606B
 *
 * Part Errata Fixes      : none
 *
 * Project Last Save Date : 10-Jul-2015 20:57:02
 *
 * Created on Date        : 10-Jul-2015 20:57:02
 *
 * Brief Description      : This  file contains  the interrupt service routine  for the Periodic Interrupt Timer
 *
 *
 *######################################################################
*/

#ifndef  _INTC_PIT_H
#define  _INTC_PIT_H
/********************  Dependent Include files here **********************/

#include "jdp.h"

/**********************  Function Prototype here *************************/

uint32_t Get_wheelRotate(void);
uint32_t Get_Speed(void);
void Set_wheelRotate(uint32_t value);
void PIT_CH0_ISR(void);
void PIT_CH1_ISR(void);


#endif  /*_INTC_PIT_H*/

/*
 *######################################################################
 *                           End of File
 *######################################################################
*/

