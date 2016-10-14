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
 * file                   : linflex_var.h
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
 * Brief Description      : This File Contains declaration of function present
 *                          in LINFLEX Var Source File
 *
 *
 *######################################################################
*/

#ifndef  _LINFLEX_VAR_H
#define  _LINFLEX_VAR_H
/********************  Dependent Include files here **********************/

#include "jdp.h"




#define  Linflex_0_LINCR2 0xFFE40030
#define  Linflex_1_LINCR2  0xFFE44030
#define  Linflex_0_BDRL    0xFFE40038
#define  Linflex_0_BDRM    0xFFE4003C
#define  Linflex_1_BDRL    0xFFE44038
#define  Linflex_1_BDRM    0xFFE4403C

#define Linflex_0_BIDR 0xFFE40034
#define Linflex_1_BIDR 0xFFE44034
#define UART_0_BDRL 0xFFE4003B
#define UART_0_BDRM 0xFFE4003F
#define UART_1_BDRL 0xFFE4403B
#define UART_1_BDRM 0xFFE4403F


 extern vuint8_t Uart0_TxBuffer[1];
 extern vuint8_t Uart0_RxBuffer[1];
 extern vuint8_t Uart1_TxBuffer[1];
 extern vuint8_t Uart1_RxBuffer[1];
 extern vuint32_t Linflex_0_TXBuffer[1];
 extern vuint32_t Linflex_0_RXBuffer[1];
 extern vuint32_t Linflex_1_TXBuffer[1];
 extern vuint32_t Linflex_1_RXBuffer[1];

#endif  /*_LINFLEX_VAR_H*/

/*
 *######################################################################
 *                           End of File
 *######################################################################
*/

