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
#include "math.h"

#define USBFS_DEVICE    (0u)


/* The buffer size is equal to the maximum packet size of the IN and OUT bulk
* endpoints.
*/
#define USBUART_BUFFER_SIZE (64u)
#define LINE_STR_LENGTH     (20u)

/* Global constants */
#define PWM_MAX         499   // PWM 100% (499) with Tpwm = 500us @ 1MHz (500 counts)
#define PWM_FACTOR      15   // 100 counts/volt with Tpwm = 500us @ 1MHz
#define VDAC_FACTOR     62.5  // 1bit/0.016mV
#define VDAC_MAX        300   // 4.8V max. with DVDAC 9 bits and PGA x2
#define POT_CONST       3100  // Max. counts to 100%

/* PI algorithm constants */
#define REFERENCE      0    // Angle reference
#define MAXINTEGRAL    4.7  // Limits the integral part to 4.7V
#define TS_FACTOR      1    // 5ms counts for Ts = 5ms

/* isr global variables */

volatile int8 factor = TS_FACTOR;    // Counter for Ts = 5ms when Timer runs faster than 5ms

/* Buttons variables */
bool idle = true;  // Signals when step input can be applied
bool run = false;  // Clock to advance Display state machine
bool save = false; // flag to preserve PI constants adjustment

/* Button sync enumerated type */
enum  DebounceState {WAIT, RUN, PRESS};

/* PID algorithm global variables */
#define KP -0.21       // Proportional default constant
#define KI -0.158      // Integral default constant @ 5ms
#define KD -0.0056     // Derivative constant
#define Ts 0.005
float KIT = Ts*KI;
float KDT = KD/Ts;
volatile float ik = 0; // Integral action and memory
volatile float dk = 0; // Derivative action and memory

int PositionSend;
float AngleSend;

void Send(float Ang) //uint16 Pwm, uint16 Pos,
{                   
    char Msg[50] = {};
    sprintf(Msg,"%f \r\n",Ang);// %d;%d Pwm,Pos,
    
    USBUART_PutString(Msg);        
}

/* Interrupt prototype */
CY_ISR_PROTO(isr_Timer_Handler);

// Interrupt handler declaration
CY_ISR(isr_Timer_Handler){
    
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

    /*                                                         */
    /* Performs the PID algorithm every 5ms when in idle state */
    /*                                                         */

    if ( (--factor <= 0) && (idle == true)) {
        factor = TS_FACTOR;

        // Reads angle value
        data = ADC_Read16();

        // Medición de sensor de ángulo
        med = ADC_CountsTo_Volts((int32) data);

        // Cálculo de ángulo en grados
        yk = med*77.4056326 + 163; //163.553089
        if(yk>180){
            yk = yk - 360;
        }
        yk = -yk;
        
        AngleSend = yk;
        
        lek = ek;
        ek = REFERENCE - yk; // Follow the reference

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
            pwm = (int16) (fabs(mk)*PWM_FACTOR);

            /* Establish motor direction*/
            if(mk > 0){
                Pin_A_Write(0);
                Pin_B_Write(1);
                //Control_Reg_Write(1);
            }
            else{
                Pin_A_Write(1);
                Pin_B_Write(0);
                //Control_Reg_Write(0);
            }
            
            /* Saturates the PWM value */
            if (pwm > PWM_MAX){ 
                pwm = PWM_MAX;
            }
            else if (pwm < 0) {
                pwm = 0; 
            }

            PWM_WriteCompare(pwm); // Outputs PWM saturated value
        #endif     
                    
        // Saturates the integral term for the next period //
        if (ik > MAXINTEGRAL)
            ik = MAXINTEGRAL;
        else
        if (ik < -MAXINTEGRAL) ik = -MAXINTEGRAL;
        
    }//if
    
} // CY_ISR

CY_ISR(InterruptTimer){
    Timer_UART_WritePeriod(500000);
    
    Send(AngleSend);
}

int main(void){
    
    uint16 count;
    uint8 buffer[USBUART_BUFFER_SIZE];
    
    //uint16 Encoder_count;
    
    uint16 Position = 0;
    
    
    /* Startup code for all blocks */
    ADC_Start();
    Timer_Start();
    Timer_UART_Start();
    PWM_Start();
    
    
    /* Start USBFS operation with 5-V operation. */
    USBUART_Start(USBFS_DEVICE, USBUART_5V_OPERATION);
    
    /* Correction for ADC_CountsTo_Volts */
    ADC_SetOffset(-1); // -1mV offset correction
    ADC_SetGain(673);  // 0.9895 gain correction

    /* PWM conditional use */
    #if defined CY_PWM_PWM_H    
        PWM_Start();
    #endif    
    
    CyGlobalIntEnable;
    
    /* Interrrup process init with StartEx not simple Start */
    isr_Timer_StartEx(isr_Timer_Handler);
    isr_1_StartEx(InterruptTimer);
    
    while(1){
            /* Host can send double SET_INTERFACE request. */
        if (0u != USBUART_IsConfigurationChanged())
        {
            /* Initialize IN endpoints when device is configured. */
            if (0u != USBUART_GetConfiguration())
            {
                /* Enumeration is done, enable OUT endpoint to receive data 
                 * from host. */
                USBUART_CDC_Init();
            }
        }
        
        /* Service USB CDC when device is configured. */
        if (0u != USBUART_GetConfiguration())
        {
            /* Check for input data from host. */
            //if (0u != USBUART_DataIsReady())
            if(1)
            {
                /* Read received data and re-enable OUT endpoint. */
                count = USBUART_GetAll(buffer);

                if (0u != count)
                {
                    /* Wait until component is ready to send data to host. */
                    while (0u == USBUART_CDCIsReady())
                    {
                    }
                    
                    /* Send data back to host. */
                    PositionSend = Position;
                    
                    if (USBUART_BUFFER_SIZE == count)
                    {
                        /* Wait until component is ready to send data to PC. */
                        while (0u == USBUART_CDCIsReady())
                        {
                        }                        
                    }
                }
            }
        }
    }
    
    
    /* Enable global interrupts. */
    
        
} // main

/* [] END OF FILE */
