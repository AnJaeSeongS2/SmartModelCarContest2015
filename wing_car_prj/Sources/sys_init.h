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
 * file                   : sys_init.h
 *
 * Target Compiler        : Codewarrior
 *
 * Target Part            : MPC5606B
 *
 * Part Errata Fixes      : none
 *
 * Project Last Save Date : 14-Jul-2015 23:19:20
 *
 * Created on Date        : 14-Jul-2015 23:19:21
 *
 * Brief Description      : This file contains system initialization code
 *
 *
 *######################################################################
*/

#ifndef  _SYS_INIT_H
#define  _SYS_INIT_H
/********************  Dependent Include files here **********************/

#include "intc_init.h"
#include "sysclk_init.h"
#include "linflex_init.h"
#include "adc_init.h"
#include "swt_init.h"
#include "flexcan_init.h"
#include "emios_init.h"
#include "dspi_init.h"
#include "siu_init.h"
#include "pit_init.h"
#include "stm_init.h"
#include "rtc_init.h"

/**********************  Function Prototype here *************************/

void sys_init_fnc (void);
extern void IVPRInitialize(void);


#endif  /*_SYS_INIT_H*/

/*
 *######################################################################
 *                           End of File
 *######################################################################
*/

