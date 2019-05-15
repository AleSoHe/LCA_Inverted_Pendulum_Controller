/* =============================================
 * PI motor control
 * Copyright Eduardo Interiano, 2017
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF Eduardo Interiano.
 * Updated by: H. Sánchez
 * 
 * Remember to set the heap to 0x200 minimun in
 * SYSTEM tab for floating point number printing
 * =============================================
*/
#include "project.h"
#include "stdbool.h"
#include "stdio.h"
#include "usbuartio.h"    

/* Global constants */
#define PWM_MAX         499   // PWM 100% (499) with Tpwm = 500us @ 1MHz (500 counts)
#define PWM_FACTOR      100   // 100 counts/volt with Tpwm = 500us @ 1MHz
#define SPEED_CHANNEL   0     // Mux input for speed feedback
#define POT_CHANNEL     1     // Mux input for PI constants setting
#define VDAC_FACTOR     62.5  // 1bit/0.016mV
#define VDAC_MAX        300   // 4.8V max. with DVDAC 9 bits and PGA x2
#define POT_CONST       3100  // Max. counts to 100%

/* PI algorithm constants */
#define REFERENCE      0    // 4krpm @ 1V por krpm
#define MAXINTEGRAL    4.7  // Limits the integral part to 4.7V
#define DEB_COUNTS     2    // 5ms counts for debounce
#define TS_FACTOR      1    // 5ms counts for Ts = 5ms

/* Global variables */
char displayStr[20] = {'\0'};

/* isr global variables */

 volatile bool select_1 = false; // Memory variables for DISPLAY_SELECT button debouncing
 volatile bool select_2 = false; 
 volatile bool select_3 = false;   // Debounced DISPLAY_SELECT button state

 volatile bool step_1 = false;   // Memory variables for START/SAVE button debouncing
 volatile bool step_2 = false; 
 volatile bool step = false;     // Debounced START/SAVE button state

 volatile int8 debounce = DEB_COUNTS; // Counter for debouncing when Timer runs faster than 10ms
 volatile int8 factor = TS_FACTOR;    // Counter for Ts = 5ms when Timer runs faster than 5ms

/* Buttons variables */
 bool idle = true;  // Signals when step input can be applied
 bool run = false;  // Clock to advance Display state machine
 bool save = false; // flag to preserve PI constants adjustment

 /* Button sync enumerated type */
 enum  DebounceState {WAIT, RUN, PRESS};

/* PID algorithm global variables */
#define KP 0.21       // Proportional default constant
#define KI 0.158      // Integral default constant @ 5ms
#define KD 0.0056     // Derivative constant
#define Ts 0.005
float KIT = Ts*KI;
float KDT = KD/Ts;
volatile float ik = 0; // Integral action and memory
volatile float dk = 0; // Derivative action and memory

/* Interrupt prototype */
CY_ISR_PROTO(isr_Timer_Handler);

// Interrupt handler declaration
CY_ISR(isr_Timer_Handler)
{

/* ISR PI algoritm local variables */
 float yk = 0; 	 // speed feedback
 float med = 0;  // medición de sensor de ángulo
 float ek = 0;   // speed error
 float lek= 0;   //last error*/ //For D control
 float mk = 0;   // total control action
 //int16 vdac = 0; // For VDAC output
 int16 data = 0; // For ADC reading

#if (PWM_Resolution)    
 int16 pwm = 0;  // For PWM output
#endif
    
    /* Debounces the switches */
    if (--debounce <= 0) {
        select_2 = select_1;
        select_1 = !(CyPins_ReadPin(DISPLAY_SELECT_0));
        select_3 = (select_1 && select_2);

        step_2 = step_1;
        step_1 = !(CyPins_ReadPin(START_0));
        step = (step_1 && step_2);
    
        /* Signals step edge for oscilloscope sync */
        if ( (step && idle) == true) 
            CyPins_SetPin(STEP_LED_0);
        else 
            CyPins_ClearPin(STEP_LED_0);
        debounce = DEB_COUNTS;
    }

    /* Performs the PID algorithm every 5ms when in idle state */
	if ( (--factor <= 0) && (idle == true)) {
		factor = TS_FACTOR;
        
        // Signals the start of the PI algorithm for time compliance measurements
        CyPins_SetPin(ISR_LED_0);

        // Reads speed value
        AMux_Select(SPEED_CHANNEL);
        data = ADC_Read16();
        
        // Medición de sensor de ángulo
	    med = ADC_CountsTo_Volts((int32) data); 
        
        // Cálculo de ángulo en grados
        yk = med*0.350194-9.28;
        if(yk>180){
            yk = yk - 360;   
        }
        yk = -yk;
        
        /* Set the reference step */
        if (step == true) { 
            lek = ek;
            ek = REFERENCE - yk; // Follow the reference
        } 
        else {
            ek = 0 - yk; // Stop the motor
            ik = 0;
            dk = 0;
        }
        
        /* PI control algorithm calculation */        
        
        /* Integral part, the rightmost term is also ik_1 */
        ik = KIT*lek + ik;
        
        /*Derivate calcutation*/
        dk=KDT*(ek-lek);
        
        /* Total PI control action */ 
        //mk = KP*ek + ik;
        
        /* Total PID control action */ 
        mk = KP*ek + ik + dk;

        /* PWM conditional use */
#if defined CY_PWM_PWM_H
        /* Scales mk to PWM range */
        pwm = (int16) (mk*PWM_FACTOR);
        
        /* Saturates the PWM value */
        if (pwm > PWM_MAX) { 
            pwm = PWM_MAX;
        }
        else if (pwm < 0) {
            pwm = 0; 
        }
        
        PWM_WriteCompare(pwm); // Outputs PWM saturated value
#endif     

        /* DVDAC conditional use */
#if defined CY_DVDAC_DVDAC_H
        /* Scales mk to DVDAC range */
        vdac = (int16) (mk*VDAC_FACTOR);
    
        // saturates the DVDAC value 
        if (vdac > VDAC_MAX) { 
            vdac = VDAC_MAX;
        }
        else if (vdac < 0) {
            vdac = 0; 
        }
        DVDAC_SetValue(vdac); // Outputs DVDAC saturated value
#endif

        // Indicates the end of the PI algorithm
        CyPins_ClearPin(ISR_LED_0);
            
        /* Saturates the integral term for the next period */
        if (ik > MAXINTEGRAL)
            ik = MAXINTEGRAL;
        else
        if (ik < -MAXINTEGRAL) ik = -MAXINTEGRAL;
	
    }
    
} // CY_ISR

/* State machine for single step button action */
bool ButtonSync(enum DebounceState* DebounceButton, bool button) { 
    
 /* Local variable */   
 bool buttonstate = false;
    
    /* Button sync */
    switch (*DebounceButton) {
        case WAIT:   
            *DebounceButton = button? RUN:WAIT;    
        break;
       
        case RUN: 
            *DebounceButton = PRESS;
            buttonstate = true;
        break;
        
        case PRESS:    
            *DebounceButton = button? PRESS:WAIT;
        break;
    
        default: *DebounceButton = WAIT;

    } // switch
        
    return buttonstate;

} // ButtonSync

int main(void){
    /* USBUART conditional use */
    #if defined CY_USBFS_USBUART_H
        
    /* USB variable definitions */
    uint16 usbcount;
    uint8  usbbuffer[USBUART_BUFFER_SIZE];
    uint8  *usbp; // pointer for usbbuffer
    char   *usbc; // pointer for displayStr

    /* Pointers to usbbuffer and displayStr for USB communication */
    usbp = &usbbuffer[0];
    usbc = &displayStr[0];

    #endif

        
    /* Startup code for all blocks */
    AMux_Start();
    ADC_Start();
    Timer_Start();
    LCD_Start();

    /* Correction for ADC_CountsTo_Volts */
    ADC_SetOffset(-1); // -1mV offset correction
    ADC_SetGain(673);  // 0.9895 gain correction

        /* QuadDec conditional use */
    #if defined CY_QUADRATURE_DECODER_QuadDec_H
        QuadDec_Start();
    #endif

        /* PWM conditional use */
    #if defined CY_PWM_PWM_H    
        PWM_Start();
    #endif    

        /* DVDAC conditional use */
    #if defined CY_DVDAC_DVDAC_H
        DVDAC_Start();
        PGA_Start();
        Opamp_Start();
    #endif    

        /* UART conditional use */
    #if defined CY_UART_UART_H 
        UART_Start();
        UART_PutString("PI Control v1.0\n\r");
    #endif

        /* USBUART conditional use */
    #if defined CY_USBFS_USBUART_H
        /* Start USBFS operation with 5-V operation. */
        USBUART_Start(USBFS_DEVICE, USBUART_5V_OPERATION);
    #endif

    /* Interrrup process init with StartEx not simple Start */
    isr_Timer_StartEx(isr_Timer_Handler);

    /* Enable global interrupts. */
    CyGlobalIntEnable;
        
} // main

/* [] END OF FILE */
