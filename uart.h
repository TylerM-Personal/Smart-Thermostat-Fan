//*****************************************************************************
//*****************************    C Source Code    ***************************
//*****************************************************************************
//   DESIGNER NAME:  Bruce Link
//
//         VERSION:  1.0
//
//       FILE NAME:  uart.h
//
//-----------------------------------------------------------------------------
//
// DESCRIPTION:
//    This module provides functions to initialize and control UART0 on
//    the MSPM0G3507 Launchpad development board. UART0 is configured
//    for a baud rate of 115200, no parity, 8 data bits, and 1 stop bit
//    (115200N81).
//
//    Pin Configuration:
//        - UART0 Tx: PA.10 (connected to XDS Rx)
//         - UART0 Rx: PA.11 (connected to XDS Tx)
//
//    Required Jumper Configuration:
//      - J25: Connects PA10 to XDS_UART
//      - J26: Connects PA11 to XDS_UART
//
//    Note: The code for this module is adapted from TI's sample project
//    'uart_rw_multibyte_fifo_poll_LP_MSPM0G3507_nortos_ticlang'.
//
//-----------------------------------------------------------------------------
// DISCLAIMER
//    This code was developed for educational purposes as part of the CSC202
//    course and is provided "as is" without warranties of any kind, whether
//    express, implied, or statutory.
//
//    The author and organization do not warrant the accuracy, completeness, or
//    reliability of the code. The author and organization shall not be liable
//    for any direct, indirect, incidental, special, exemplary, or consequential
//    damages arising out of the use of or inability to use the code, even if
//    advised of the possibility of such damages.
//
//    Use of this code is at your own risk, and it is recommended to validate
//    and adapt the code for your specific application and hardware
//    requirements.
//
// Copyright (c) 2024 by Bruce Link
//    You may use, edit, run or distribute this file as long as the above
//    copyright notice remains
// *****************************************************************************
//******************************************************************************


#ifndef __UART_H__
#define __UART_H__

//-----------------------------------------------------------------------------
// Loads standard C include files
//-----------------------------------------------------------------------------
#include <stdint.h>


// --------------------------------------------------------------------------
// Prototype for Launchpad support functions
// --------------------------------------------------------------------------

void UART_init(uint32_t baud_rate);
char UART_in_char(void);
void UART_out_char(char data);



#endif /* __UART_H__ */
