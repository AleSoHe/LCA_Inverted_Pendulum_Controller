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
#define PWM_FACTOR      200   // 100 counts/volt with Tpwm = 500us @ 1MHz
#define VDAC_FACTOR     62.5  // 1bit/0.016mV
#define VDAC_MAX        300   // 4.8V max. with DVDAC 9 bits and PGA x2
#define POT_CONST       3100  // Max. counts to 100%

/* PI algorithm constants */
#define REFERENCE      0 //-0.15    // Angle reference
#define PosREFERENCE   0   //-0.15    // Position reference
#define TS_FACTOR      1    // 5ms counts for Ts = 5ms
#define N              0.3    // Filter
/* isr global variables */

volatile int8 factor = TS_FACTOR;    // Counter for Ts = 5ms when Timer runs faster than 5ms

/* Buttons variables */
bool idle = true;  // Signals when step input can be applied
bool save = false; // flag to preserve PI constants adjustment

/* Button sync enumerated type */
enum  DebounceState {WAIT, RUN, PRESS};

/* PID algorithm global variables */
#define KPpos -0.005       // Proportional default constant

#define K_ang 		-13.9828    // -0.21       
#define K_vel_ang 	-4.5569     // -0.0158      
#define K_accel_ang -0.8372	    // -0.0056       

#define Ts 0.005

int PositionSend;
float AngleSend;

// Control parameters
float u = 0;			 // Input
float angle = 0; 	     // Angle
float vel_angle = 0;     // Angle velocity
float accel_angle = 0; 	 // Angle acceleration
float l_angle = 0; 	   	 // Last angle
float l_vel_angle = 0;   // Last angle velocity
float yk;

float angles[8] = {0,0,0,0,0,0,0,0};

int i;                  // Integer for for loops
int mov_av_N = 4;       // Moving average number

void Send(float Ang, int Pos) //uint16 Pwm, uint16 Pos,
{                   
    char Msg[50] = {};
    sprintf(Msg,"%f;%d \r\n",Ang,Pos);// %d;%d Pwm,Pos,
    
    USBUART_PutString(Msg);        
}

/* Interrupt prototype */
CY_ISR_PROTO(isr_Timer_Handler);

// Interrupt handler declaration
CY_ISR(isr_Timer_Handler){
    //Ex_time_Write(1);
    /* ISR PI algoritm local variables */

	
    float med = 0;  // medición de sensor de ángulo
	int16 data = 0; // For ADC reading    
    int Position = 0;

    #if (PWM_Resolution)    
        int16 pwm = 0;  // For PWM output
    #endif

    /*                                      */
    /* Performs the state feedback control	*/
    /*                                      */

    if ( (--factor <= 0) && (idle == true)) {
        factor = TS_FACTOR;
        
        // Reads angle value
        data = ADC_Read16();
        
        // Reads position value
        Position = QuadDec_GetCounter();
        if(Position > 32767){
            Position = Position -65535;   
        }
        PositionSend = Position;
        
        // Medición de sensor de ángulo
        med = ADC_CountsTo_Volts((int32) data);
        
        // Cálculo de ángulo en grados
        yk = med*77.4056326 + 157; //161.6 //157.553089
        if(yk>180){
            yk = yk - 360;
        }
        yk = -yk; 
        
        // Moving Average

        for (i=0; i<mov_av_N-1; i=i+1){
            angles[i] = angles[i+1];
        }
        angles[mov_av_N-1] = yk;
        
        // Filtered yk
        yk = 0;
        for (i=0; i<mov_av_N; i=i+1){
            yk = yk + angles[i];
        }
        yk = yk/mov_av_N;
        
        l_angle = angle;
        l_vel_angle = vel_angle;
        
        angle = yk;
        vel_angle = (angle - l_angle);
        accel_angle = (vel_angle - l_vel_angle);

        // Control action
        u = REFERENCE - angle*K_ang - vel_angle*K_vel_ang - accel_angle*K_accel_ang; // Follow the reference
        
        /* PWM conditional use */
        #if defined CY_PWM_PWM_H
            /* Scales mk to PWM range */
            pwm = 200 + (int16) (fabs(u)*PWM_FACTOR); //255+
            AngleSend = vel_angle;
            
            /* Establish motor direction*/
            if(u > 0){
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
            
            //if (vel_angle < 0.15 && fabs(angle)<2){ 
            //    pwm = 0;
            //}
            
            
            PWM_WriteCompare(pwm); // Outputs PWM saturated value
        #endif     
    }//if
    
} // CY_ISR

CY_ISR(InterruptTimer){
    Timer_UART_WritePeriod(5000);
    Send(AngleSend,PositionSend);//Send(AngleSend);
}

int main(void){
    
    uint16 count;
    uint8 buffer[USBUART_BUFFER_SIZE];
    
    //uint16 Encoder_count;
    
    /* Startup code for all blocks */
    ADC_Start();
    Timer_Start();
    Timer_UART_Start();
    PWM_Start();
    QuadDec_Start();
    
    /* Start USBFS operation with 5-V operation. */
    USBUART_Start(USBFS_DEVICE, USBUART_5V_OPERATION);
    
    /* Correction for ADC_CountsTo_Volts */
    //ADC_SetOffset(-1); // -1mV offset correction
    //ADC_SetGain(673);  // 0.9895 gain correction

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
