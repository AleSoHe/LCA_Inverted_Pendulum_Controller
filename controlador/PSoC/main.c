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

/* Global constants */
#define PWM_MAX         499   // PWM 100% (499) with Tpwm = 500us @ 1MHz (500 counts)
#define PWM_FACTOR      100   // 100 counts/volt with Tpwm = 500us @ 1MHz
#define VDAC_FACTOR     62.5  // 1bit/0.016mV
#define VDAC_MAX        300   // 4.8V max. with DVDAC 9 bits and PGA x2
#define POT_CONST       3100  // Max. counts to 100%

/* PI algorithm constants */
#define REFERENCE      0    // 4krpm @ 1V por krpm
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
        yk = med*0.350194-9.28;
        if(yk>180){
            yk = yk - 360;
        }
        yk = -yk;

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
            }
            else{
                Pin_A_Write(1);
                Pin_B_Write(0);
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

int main(void){
        
    /* Startup code for all blocks */
    ADC_Start();
    Timer_Start();
    PWM_Start();
    
    /* Correction for ADC_CountsTo_Volts */
    ADC_SetOffset(-1); // -1mV offset correction
    ADC_SetGain(673);  // 0.9895 gain correction

        /* PWM conditional use */
    #if defined CY_PWM_PWM_H    
        PWM_Start();
    #endif    

    /* Interrrup process init with StartEx not simple Start */
    isr_Timer_StartEx(isr_Timer_Handler);

    /* Enable global interrupts. */
    CyGlobalIntEnable;
        
} // main

/* [] END OF FILE */
