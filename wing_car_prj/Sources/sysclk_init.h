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
 * file                   : sysclk_init.h
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
 * Brief Description      : The file contains declaration for Mode Entry, System Clock,
 *                          CMU initialization functions.
 *
 *
 *######################################################################
*/

#ifndef  _SYSCLK_INIT_H
#define  _SYSCLK_INIT_H

/********************  Dependent Include files here **********************/

#include "jdp.h"

/******************************************************************************
* Global functions
******************************************************************************/

void sysclk_module_init_fnc(void);
void mode_entry_init_fnc(void);
void mode_entry_post_config_fnc(void);
void sysclk_init_fnc(void);
void cmu_init_fnc (void);
void mode_entry_post_config_fnc (void);


#endif  /*_SYSCLK_INIT_H*/

/*
 *######################################################################
 *                           End of File
 *######################################################################
*/

