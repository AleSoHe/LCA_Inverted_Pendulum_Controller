/*******************************************************************************
* File Name: usbuartio.h
* Version 1.00
*
*  Description:
*   Provides the function definitions for the usb_uart functions.
*
*
********************************************************************************
* Based on a Cypress example. Eduardo Interiano 2017
*
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/
#if !defined(usbuartio_H)
#define usbuartio_H


#include <project.h>
#include "stdbool.h"
    
/* usbuartio API. */
void USB_Uart_Init(void);
uint16 USB_Uart_Read(uint8 *data[]);
bool USB_Uart_Print(char* pStrBuf[]);
bool USB_Uart_PrintLn(char* pStrBuf[]);
void USB_Uart_Write(uint16 count, uint8 *buffer[]);
void USB_Uart_DisplayStatus(void);

/* usbuartio Constants */

#define USBFS_DEVICE    (0u)

/* The buffer size is equal to the maximum packet size of the IN and OUT bulk
* endpoints.
*/
#define USBUART_BUFFER_SIZE (64u)
#define LINE_STR_LENGTH     (16u)


#endif /* usbuartio_H */


/* [] END OF FILE */

