/*******************************************************************************
* File Name: usbuartio.c
*
* Version: 1.0
*
* Description:
*
*   This library has functions to:
*
*   Init USB, the component is enumerated as a Virtual Com port
*   Receive data from the USB
*   Transmit data through USB
*   Display the line settings in the LCD for the PSoC3/PSoC5LP
*
* Related Document:
*  Universal Serial Bus Specification Revision 2.0
*  Universal Serial Bus Class Definitions for Communications Devices
*  Revision 1.2
*
********************************************************************************
* Based on a Cypress example, Eduardo Interiano 2017
*
* Copyright 2015, Cypress Semiconductor Corporation. All rights reserved.
* This software is owned by Cypress Semiconductor Corporation and is protected
* by and subject to worldwide patent and copyright laws and treaties.
* Therefore, you may use this software only as provided in the license agreement
* accompanying the software package from which you obtained this software.
* CYPRESS AND ITS SUPPLIERS MAKE NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
* WITH REGARD TO THIS SOFTWARE, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT,
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
*******************************************************************************/

#include <project.h>
#include "usbuartio.h"
#include "stdio.h"
#include "stdbool.h"

#if defined (__GNUC__)
    /* Add an explicit reference to the floating point printf library */
    /* to allow usage of the floating point conversion specifiers. */
    /* This is not linked in by default with the newlib-nano library. */
    asm (".global _printf_float");
#endif

/* USBUART conditional use */
#if defined CY_USBFS_USBUART_H
    
/*******************************************************************************
* Function Name: USB_Uart_Init
********************************************************************************
*
* Summary:
*  This  function performs the following actions:
*   1. Waits until VBUS becomes valid and starts the USBFS component which is
*      enumerated as virtual Com port.
*   2. Waits until the device is enumerated by the host.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void USB_Uart_Init(void)
{   
    /* Host can send double SET_INTERFACE request. */
    if (0u != USBUART_IsConfigurationChanged())
    {
        /* Initialize IN endpoints when device is configured. */
        if (0u != USBUART_GetConfiguration())
        {
            /* Enumeration is done, enable OUT endpoint to receive data from host. */
            USBUART_CDC_Init();
        }
    }
}

/*******************************************************************************
* Function Name: USB_Uart_Read
********************************************************************************
*
* Summary:
*  This function performs the following action:
*   1. Looks for data coming from the hyper terminal to read it
*
* Parameters:
*  None.
*
* Return:
*  received byte count
*
*******************************************************************************/
uint16 USB_Uart_Read(uint8 *data[])
{        
    uint16 count = 0;
        
    /* Service USB CDC when device is configured. */
    if (0u != USBUART_GetConfiguration())
    {
        /* Check for input data from host. */
        if (0u != USBUART_DataIsReady())
        {
            /* Read received data and re-enable OUT endpoint. */
            count = USBUART_GetAll(*data);
        }
    }
    
    return count;  
    
}

/*******************************************************************************
* Function Name: USB_Uart_Write
********************************************************************************
*
* Summary:
*  This function performs the following action:
*   1. Waits until the component is ready to send data to host and sends it
*
* Parameters:
*  byte count and data to transmit
*
* Return:
*  None.
*
*******************************************************************************/
void USB_Uart_Write(uint16 count, uint8 *buffer[])
{

    if (0u != count)
    {
        /* Wait until component is ready to send data to host. */
        while (0u == USBUART_CDCIsReady()) {
        }

        /* Send data to host. */
        USBUART_PutData(*buffer, count);

        /* If the last sent packet is exactly the maximum packet 
         *  size, it is followed by a zero-length packet to assure
         *  that the end of the segment is properly identified by 
         *  the terminal.
         */
        if (USBUART_BUFFER_SIZE == count)
        {
            /* Wait until component is ready to send data to PC. */
            while (0u == USBUART_CDCIsReady()) {
            }

            /* Send zero-length packet to PC. */
            USBUART_PutData(NULL, 0u);
        }
        
    }        
   
}

/*******************************************************************************
* Function Name: USB_Uart_Print
********************************************************************************
*
* Summary:
*  This function performs the following action:
*   1. Waits max 5ms until the component is ready to send a null terminated string
*      to the host and sends it
*
* Parameters:
*  char string pointer
*
* Return:
*  true if succeds, false if not.
*
*******************************************************************************/
bool USB_Uart_Print(char* pStrBuf[])
{
uint count = 0;
    
    /* Wait max 5 ms until component is ready to send data to host. */
    while (0u == USBUART_CDCIsReady() && count < 5) {
        CyDelay(1);
        count ++;
    }

    /* Send data to host. */
    if (1u == USBUART_CDCIsReady()) {
        USBUART_PutString(*pStrBuf);
        return true;
    }
    // timeout
    else return false;
}

/*******************************************************************************
* Function Name: USB_Uart_PrintLn
********************************************************************************
*
* Summary:
*  This function performs the following action:
*   1. Waits max 5 ms until the component is ready to send a null terminated string
*      to the host and sends it with a CRLF appended at the end. 
*
* Calls USB_Uart_Print()
*
* Parameters:
*  char string pointer
*
* Return:
*  true if succeds, false if not.
*
*******************************************************************************/
bool USB_Uart_PrintLn(char* pStrBuf[])
{

    strncat(*pStrBuf,"\n\r",2);
    return USB_Uart_Print(pStrBuf);

}

/*******************************************************************************
* Function Name: USB_Uart_DisplayStatus
********************************************************************************
*
* Summary:
*  This function performs the following action:
*   1. PSoC3/PSoC5LP: the LCD shows the line settings.
*
* Parameters:
*  none
*
* Return:
*  None.
*
*******************************************************************************/
void USB_Uart_DisplayStatus(void)
{
    
#if (CY_PSOC3 || CY_PSOC5LP)
        
    uint8 state;
    char8 lineStr[LINE_STR_LENGTH];    

    char8* parity[] = {"None", "Odd", "Even", "Mark", "Space"};
    char8* stop[]   = {"1", "1.5", "2"};

    /* Check for Line settings change. */
    state = USBUART_IsLineChanged();
    if (0u != state)
    {
        /* Output on LCD Line Coding settings. */
        if (0u != (state & USBUART_LINE_CODING_CHANGED))
        {                  
            /* Get string to output. */
            sprintf(lineStr,"BR:%4ld %d%c%s", USBUART_GetDTERate(),\
            (uint16) USBUART_GetDataBits(),\
            parity[(uint16) USBUART_GetParityType()][0],\
            stop[(uint16) USBUART_GetCharFormat()]);

            /* Clear LCD line. */
            LCD_Position(0u, 0u);
            LCD_PrintString("                    ");

            /* Output string on LCD. */
            LCD_Position(0u, 0u);
            LCD_PrintString(lineStr);
        }

        /* Output on LCD Line Control settings. */
        if (0u != (state & USBUART_LINE_CONTROL_CHANGED))
        {                   
            /* Get string to output. */
            state = USBUART_GetLineControl();
            sprintf(lineStr,"DTR:%s,RTS:%s",  (0u != (state & USBUART_LINE_CONTROL_DTR)) ? "ON" : "OFF",
                                              (0u != (state & USBUART_LINE_CONTROL_RTS)) ? "ON" : "OFF");

            /* Clear LCD line. */
            LCD_Position(1u, 0u);
            LCD_PrintString("                    ");

            /* Output string on LCD. */
            LCD_Position(1u, 0u);
            LCD_PrintString(lineStr);
        }
    }
#endif /* (CY_PSOC3 || CY_PSOC5LP) */
}

#endif

/* [] END OF FILE */
