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
#define PWM_FACTOR      300   // 100 counts/volt with Tpwm = 500us @ 1MHz
#define VDAC_FACTOR     62.5  // 1bit/0.016mV
#define VDAC_MAX        300   // 4.8V max. with DVDAC 9 bits and PGA x2
#define POT_CONST       3100  // Max. counts to 100%

/* PI algorithm constants */
#define REFERENCE      -0.25 //-0.15    // Angle reference
#define PosREFERENCE    0   //-0.15    // Position reference
#define MAXINTEGRAL    4.7  // Limits the integral part to 4.7V
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
#define KPpos -0.05       // Proportional default constant

#define K_ang 		-0.21    //-0.6        // Proportional default constant
#define K_vel_ang 	-0.158   //-0.158      // Integral default constant @ 5ms
#define K_accel_ang 	-0.0056    // Derivative constant

#define Ts 0.005
float KIT = Ts*KI;
float KDT = KD/Ts;
volatile float ik = 0; // Integral action and memory
volatile float dk = 0; // Derivative action and memory

int PositionSend;
float AngleSend;

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
    float u = 0;			 // Input
    float angle = 0; 	     // Angle
	float vel_angle = 0;     // Angle velocity
	float accel_angle = 0; 	 // Angle acceleration
	float l_angle = 0; 	   	 // Last angle
	float l_vel_angle = 0;   // Last angle velocity
	float l_accel_angle = 0; // Last angle acceleration
	
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
        
        //Position error 
        epk = PosREFERENCE - Position;
        
        // Medición de sensor de ángulo
        med = ADC_CountsTo_Volts((int32) data);
        
        // Cálculo de ángulo en grados
        yk = med*77.4056326 + 157; //161.6 //157.553089
        if(yk>180){
            yk = yk - 360;
        }
        yk = -yk; 
        
        l_angle = angle;
        l_vel_angle = vel_angle;
        
        angle = yk;
        vel_angle = (angle - l_angle)/Ts;
        accel_angle = (vel_angle - l_vel_angle)/Ts;

        u = REFERENCE - angle*K_ang - vel_angle*K_vel_ang - accel_angle*K_accel_ang; // Follow the reference
        
        /* PI control algorithm calculation */        

        /* Integral part, the rightmost term is also ik_1 */
        ik = KIT*lek + ik;

        /*Derivate calcutation*/
        
        dk=KDT*N*(ek-lek)-lmk*(N*Ts-1);
        
        //dk=KDT*(ek-lek);

        /* Total PI control action */ 
        //mk = KP*ek + ik;

        /* Total PID control action */
        mk = KPpos*epk; //mk = KP*ek + ik + dk + KPpos*epk; //
        
        /* PWM conditional use */
        #if defined CY_PWM_PWM_H
            /* Scales mk to PWM range */
            pwm = 220 + (int16) (fabs(mk)*PWM_FACTOR); //255+

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
            PWM_WriteCompare(pwm); // Outputs PWM saturated value
        #endif     
                    
        // Saturates the integral term for the next period //
        if (ik > MAXINTEGRAL)
            ik = MAXINTEGRAL;
        else
        if (ik < -MAXINTEGRAL) ik = -MAXINTEGRAL;
    //Ex_time_Write(0);    
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
