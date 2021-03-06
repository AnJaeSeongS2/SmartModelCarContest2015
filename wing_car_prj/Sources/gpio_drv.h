/*
 *########################################################################
 *                (c) Copyright 2013 Freescale Semiconductor, Inc.
 *                         ALL RIGHTS RESERVED. 
 *########################################################################
 * 
 * Brief Description     : GPIO driver header file
 *
 **************************************************************************
*/

#ifndef _GPIO_DRV_H
#define _GPIO_DRV_H /* Only include header file once */
/******************************************************************************
* Includes
******************************************************************************/
#include "typedefs.h"
#include "jdp.h"

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/
/* Example defines */
#define PORTA_0     (uint16_t)0
#define PORTA_1     (uint16_t)1
#define PORTA_2     (uint16_t)2
#define PORTA_3     (uint16_t)3
#define PORTA_4     (uint16_t)4
#define PORTA_5     (uint16_t)5
#define PORTA_6     (uint16_t)6
#define PORTA_7     (uint16_t)7


/******************************************************************************
* Types
******************************************************************************/

/******************************************************************************
* Local Functions
******************************************************************************/

/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* Static variables
******************************************************************************/

/******************************************************************************
* Global functions
******************************************************************************/
uint8_t GPIO_GetState (uint16_t);			/* return: Current Value of GPIO Pin; paremter: Channel Number */
void GPIO_SetState (uint16_t, uint8_t); /* parameters: Channel Number, Value to set output */ 
 
#endif /* End of Header File Define */

/*
 *######################################################################
 *                           End of File
 *######################################################################
*/