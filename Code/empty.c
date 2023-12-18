/*
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== empty.c ========
 */
/* XDCtools Header files */



#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
// #include <ti/drivers/I2C.h>
// #include <ti/drivers/SDSPI.h>
// #include <ti/drivers/SPI.h>
// #include <ti/drivers/UART.h>
// #include <ti/drivers/Watchdog.h>
// #include <ti/drivers/WiFi.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "driverlib/PWM.h"
#include "utils/uartstdio.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/timer.h"


//Milestone 10
#include <ti/sysbios/knl/Clock.h>

extern const ti_sysbios_knl_Semaphore_Handle pidSemaphore;


#define SENSOR_PORT GPIO_PORTB_BASE
#define SENSOR_PIN  GPIO_PIN_3


#define MS_TO_LOOPS(ms) ((SysCtlClockGet() * (ms)) / (3 * 1000))
#define pwmMax 6250
//#define PWM_GEN_0 0x00000040
#define HIGH 0x01

uint32_t LeftDutyCycle = 100;
uint32_t RightDutyCycle = 100;


// Global variables for PID control
float integral = 0;  //module 6
float previousError = 0;  //module 6


int onVal = 0;





//Milestone 9

#define BUFFER_SIZE 20
int pingBuffer[BUFFER_SIZE];
int pongBuffer[BUFFER_SIZE];
int *currentBuffer = pingBuffer;
int *nextBuffer = pongBuffer;
int bufferIndex = 0;
int pidCounter = 0;  // Add this counter

int collectData = 0;  // Global variable to control data collection. 0 = don't collect, 1 = collect


//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Hello World (hello)</h1>
//!
//! A very simple ``hello world'' example.  It simply displays ``Hello World!''
//! on the UART and is a starting point for more complicated applications.
//!
//! UART0, connected to the Virtual Serial Port and running at
//! 115,200, 8-N-1, is used to display messages from this application.
//
//*****************************************************************************

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
volatile uint32_t decayCounter = 0;

/*
uint32_t adc_rd(void){
    uint32_t adc0Val[8];
    ADCIntClear(ADC0_BASE, 1);
    ADCProcessorTrigger(ADC0_BASE, 1);
    while(!ADCIntStatus(ADC0_BASE, 1, false)) {};
    ADCSequenceDataGet(ADC0_BASE, 1, adc0Val);
    return adc0Val[0];


}
*/
//functions predefined on top so that anyone can call each function
uint32_t readReflectanceSensor();
void ConfigureTimer40msB();
void handler50ms() ;
void basicPIDControl();
void deadEnd();
void adjustMotorOutput(float output);

void transmitData();
void swapBuffers();
void acquireData(float error);



uint32_t adc_rd(void){
    uint32_t adc0Val[8];
    ADCIntClear(ADC0_BASE, 1);
    ADCProcessorTrigger(ADC0_BASE, 1);
    while(!ADCIntStatus(ADC0_BASE, 1, false)) {};
    ADCSequenceDataGet(ADC0_BASE, 1, adc0Val);
    return adc0Val[0];


}



uint32_t adc_fwd(void)
{
    uint32_t adc0Val[8];
    ADCIntClear(ADC0_BASE, 1);
    ADCProcessorTrigger(ADC0_BASE, 1);
    while (!ADCIntStatus(ADC0_BASE, 1, false))
    {
    };
    ADCSequenceDataGet(ADC0_BASE, 1, adc0Val);
    return adc0Val[2];
}

void deadEnd(void)
{
    while (1)
    {
        Semaphore_pend(pidSemaphore, BIOS_WAIT_FOREVER);
        int frontDistance = (int)adc_fwd();

        if(onVal == 1) {
            // Green LED logic: If the robot is far enough from the wall (> 10cm, or ADC reading above 1000)
            if (frontDistance <= 1000)
            {
                //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1, 0x0); // Turn off all LEDs
                //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);                    // Green LED on
            }
            // Yellow LED logic: If the robot is a little close to the wall (< 10cm, ADC reading between 1000 and 3000)
            else if (frontDistance > 1000 && frontDistance < 2300)
            {
                //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1, 0x0);        // Turn off all LEDs
                //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3 | GPIO_PIN_1, GPIO_PIN_3 | GPIO_PIN_1); // Yellow (Green + Red) LED on
            }
            // Red LED logic + U-turn: If the robot is too close to the wall (< 7cm, or ADC reading less than or equal to 3000)
            else if (frontDistance >= 2300)
            {
                // Turn off all LEDs and turn on the red LED
                //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1, 0x0);
                //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);

                // Stop current movement
                PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, false);
                PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, false);

                // Perform U-turn
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, pwmMax);  // u- turn left
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, pwmMax);
                GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, 0); // Assuming this controls the direction of one motor
                GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, 0); // And this controls the direction of the other motor

                // Ensure PWM outputs are enabled
                PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
                PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);


                // Loop to ensure robot keeps moving until it has cleared the obstacle
                while(adc_fwd() > 1000)
                {
                    SysCtlDelay(SysCtlClockGet() / 3 / 1000 * 10); // Delay for 10 milliseconds
                }

                // Restore normal motion. Create a global var set as 0 then in str bluetooth make it one and have the if statement for the lines below


                    LeftDutyCycle = 100;
                    RightDutyCycle = 100;
                    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, LeftDutyCycle * pwmMax / 100);
                    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, RightDutyCycle * pwmMax / 100);
                    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_PIN_2);
                    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, 0x0);
            }
        }
    }
}



void adjustMotorOutput(float output){
    // Define constants

    if(output > 0.99) {
        output = 0.99;
    }
    else if(output < -0.99) {
        output = -0.99;
    }

    if(output < 0) {
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, (1 + output)*LeftDutyCycle * pwmMax / 100);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, RightDutyCycle * pwmMax / 100);
    }
    else if(output >= 0) {
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, RightDutyCycle * pwmMax / 100);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (1 - output)*LeftDutyCycle * pwmMax / 100);


    }


}





//mile stone 8
// Enumeration for the type of lines
typedef enum {
    NO_LINE,
    FIRST_THIN_LINE,
    SECOND_THIN_LINE,
    THICK_BLACK_LINE
} LineType;

static LineType last_detected_line = NO_LINE;


uint32_t readReflectanceSensor(void) {
    // Reset the decay counter
    decayCounter = 0;

    // Start the 40ms timer for WTimer1B
    TimerDisable(WTIMER1_BASE, TIMER_B);



    // Configure the pin as output and set it to high (charge the capacitor)
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_PIN_3);

    // Wait at least 10 Micros
    SysCtlDelay(SysCtlClockGet() / (3 * 100000));

    // Configure the pin as input (high impedance)
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_3);

    // Wait for the I/O line to go low
    while(GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_3) & GPIO_PIN_3) {
        // The handler40ms will keep increasing the decayCounter every 40ms
        // Do nothing here, just wait for the line to go low
        decayCounter++;
    }

    // Stop the 40ms timer for WTimer1B


    uint32_t ui32Period = SysCtlClockGet() / 1000 * 40 - 1;  // 40ms period



    // Set the load value for Timer1B
    TimerLoadSet(WTIMER1_BASE, TIMER_B, ui32Period - 1);

    TimerEnable(WTIMER1_BASE, TIMER_B);


    return decayCounter;  // This will return how many 40ms ticks occurred before the line went low
}

//milestone 10


void FlashRedLEDForOneMinute(void) {
    uint32_t halfSecond = SysCtlClockGet() / 2;  // Calculate delay for half a second at 50 MHz

    // Assuming that your SysCtlDelay() function delays for 3 clock cycles,
    // the delay count should be adjusted accordingly:
    uint32_t delayCount = halfSecond / 3;
    int i;
    int j;
    for (i = 0; i < 60; i++) {  // 60 seconds
        for (j = 0; j < 2; j++) {  // Blink twice per second
            // Toggle LED
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1) ^ GPIO_PIN_1);

            // Delay for half a second
            SysCtlDelay(delayCount);
        }
    }

    // Turn off the LED after blinking
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
}


static uint32_t startTime = 0;
static uint32_t stopTime = 0;

void startTimer() {
    // Get the current clock count
    startTime = Clock_getTicks();
}

void stopTimerAndSendData() {
    // Get the current clock count
    stopTime = Clock_getTicks();

    // Calculate the elapsed time in milliseconds
    uint32_t elapsedTime = ((stopTime - startTime) * Clock_tickPeriod / 1000)*2; // Convert to milliseconds

    // Calculate the elapsed time in seconds and then in hundredths of a second
    uint32_t timeInHundredths = (uint32_t)(elapsedTime * 100 / 1000); // Convert milliseconds to hundredths of seconds

    // Extract seconds and hundredths for separate printing
    uint32_t seconds = timeInHundredths / 100;
    uint32_t hundredths = timeInHundredths % 100;

    // Send the elapsed time to the PC with two decimal places
    UARTprintf("Elapsed time: %u.%02u seconds\n", seconds, hundredths);




}



void handleLineDetection(void) {
    uint32_t cycles = readReflectanceSensor();
    LineType detected_line;

    if(onVal == 1) {
        // Based on threshold
        if(cycles > 3000) {
            // Check again for thick line
            SysCtlDelay(SysCtlClockGet() / 33);  // 160ms delay
            cycles = readReflectanceSensor();

            if(cycles > 3000) {
                // It's a thick line, flash the red led for a minute and then close the bios after that.
                //shut the bot down
                PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, false);
                PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, false);

                stopTimerAndSendData();

                FlashRedLEDForOneMinute();

                // Ensure the LED is turned off before stopping BIOS
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1, 0x0);

                // Stop the BIOS
                BIOS_exit(0);  // The argument can be any integer, typically used to indicate the reason for shutdown.





                detected_line = THICK_BLACK_LINE;  // Update the detected line
            } else {
                // It's a thin line, determine which one based on history
                if(last_detected_line == NO_LINE) {
                    detected_line = FIRST_THIN_LINE;
                    //milestone 9
                    collectData = 1;  // Start data collection

                    startTimer();

                    // Directly enable Green LED
                    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1, 0x0);
                    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
                } else {
                    detected_line = SECOND_THIN_LINE;
                    //milestone 9
                    collectData = 0;  // Stop data collection

                    //milestone 10
                    //Turn on the RED LED

                    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1, 0x0);
                    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);

                }
            }
        } else {
            // If no line detected, just assign the last detected line state
            detected_line = last_detected_line;
        }

        // Update the last detected line state
        last_detected_line = detected_line;
    }

}


//milestone 9



void transmitData() {
    int i;


    // Send data from the full buffer (nextBuffer)
    for (i = 0; i < BUFFER_SIZE; i++) {
        UARTprintf("%i ", nextBuffer[i]);  // Sends each value followed by a space
    }

    // Send Carriage Return and Linefeed to signify end of frame
    UARTprintf("\n");  // Sends each value followed by a space

}

void swapBuffers() {
    if (currentBuffer == pingBuffer) {
        //Turn on the blue LED when it enter the Ping part of it
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1, 0x0);
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
        currentBuffer = pongBuffer;
        nextBuffer = pingBuffer;
    } else {
        //Turn on the yellow LED when it enter the Pong part of it

        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1, 0x0);
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
        currentBuffer = pingBuffer;
        nextBuffer = pongBuffer;
    }
}

void acquireData(float error) {
    if (bufferIndex < BUFFER_SIZE) {
        currentBuffer[bufferIndex++] = (int)(error * 100);  // Multiply by 100 to convert to percentage
    } else {
        bufferIndex = 0;
        swapBuffers();  // Switches between ping and pong
        transmitData(); // Sends data to PC
    }
}


//Milestone 9 above



void basicPIDControl(void){
    while(1) {
        Semaphore_pend(pidSemaphore, BIOS_WAIT_FOREVER);

        int desiredPosition = 1800;  // Hypothetical desired position (setpoint)
        int currentPosition = (int)adc_rd();  // Measured position (process variable)

        // Tune your PID constants (Kp, Ki, Kd) as per your requirement.
        float Kp = 1.5, Ki = 0, Kd = 0;

        float output ;
        float error = ((float)desiredPosition - currentPosition) / desiredPosition ;  // Calculate error percent in decimal
        output = Kp*error ;  // PID output

        previousError = error;

        adjustMotorOutput(output);

        //milestone 9
        if(collectData == 1) {
            pidCounter++;
            if (pidCounter == 1) {  // Only acquire data every second call
                pidCounter = 0;
                acquireData(error);  // Call data acquisition logic
            }
        }
        int j;
        if (last_detected_line == SECOND_THIN_LINE) {
                // Transmit remaining data in currentBuffer up to bufferIndex
                for (j = 0; j < bufferIndex; j++) {
                    UARTprintf("%i ", currentBuffer[j]);
                }
                UARTprintf("\r\n");  // End the transmission

                bufferIndex = 0;  // Empty the buffer
            }



/*
        UARTprintf("%i\n", currentPosition);*/

    }




}








void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //


    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); //milestone2




    //
    // Enable UART0
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1); //milestone2

    //
    // Configure GPIO Pins for UART mode.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PB0_U1RX); //milestone2
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinConfigure(GPIO_PB1_U1TX); //milestone2

    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1); //milestone2

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    // Enable the UART interrupt
    IntEnable(INT_UART1);
    UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);

        // Enable processor interrupts

    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |UART_CONFIG_PAR_NONE));
    //ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |UART_CONFIG_PAR_NONE));
    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(1, 115200, SysCtlClockGet());
}


/*
 *  ======== main ========
 */



void handler50ms(void) {
    // Clear the timer interrupt
        TimerIntClear(WTIMER0_BASE, TIMER_TIMA_TIMEOUT);
        Semaphore_post(pidSemaphore);

}

void ConfigureTimer50ms(void)
{
    // Assuming a 50MHz system clock (adjust accordingly)
    uint32_t ui32Period = SysCtlClockGet()/1000 * 50 - 1;  // 50ms period


    // Enable the timer peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);

    // Configure Timer0 to be periodic
    TimerConfigure(WTIMER0_BASE, TIMER_CFG_A_PERIODIC | TIMER_CFG_SPLIT_PAIR );

    // Set the load value for Timer0
    TimerLoadSet(WTIMER0_BASE, TIMER_A, ui32Period -1);

    // Enable the timer interrupt in the NVIC
    IntEnable(INT_WTIMER0A);
    TimerIntEnable(WTIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Enable Timer0
    TimerEnable(WTIMER0_BASE, TIMER_A);
}



void handler40msB(void) {
    // Clear the timer interrupt
    TimerIntClear(WTIMER1_BASE, TIMER_TIMB_TIMEOUT);
    // Here, you can set a flag or post a semaphore, etc., based on your application needs
/*
    uint32_t val;
    val = readReflectanceSensor();
    UARTprintf("%i\n", val);
*/

    handleLineDetection();


}

void ConfigureTimer40msB(void) {
    // Assuming a 50MHz system clock (adjust accordingly)
    uint32_t ui32Period = SysCtlClockGet() / 1000 * 40 - 1;  // 40ms period

    // Enable the timer peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER1);

    // Configure Timer1B to be periodic
    TimerConfigure(WTIMER1_BASE, TIMER_CFG_B_PERIODIC | TIMER_CFG_SPLIT_PAIR);

    // Set the load value for Timer1B
    TimerLoadSet(WTIMER1_BASE, TIMER_B, ui32Period - 1);

    // Enable the timer interrupt in the NVIC
    IntEnable(INT_WTIMER1B);
    TimerIntEnable(WTIMER1_BASE, TIMER_TIMB_TIMEOUT);

    // Enable Timer1B
    TimerEnable(WTIMER1_BASE, TIMER_B);
}



void handlerBT(void) {
        uint32_t ui32Status;

        // Get the interrrupt status
        ui32Status = UARTIntStatus(UART1_BASE, true);

        // Clear the asserted interrupts
        UARTIntClear(UART1_BASE, ui32Status);

        if (UARTCharsAvail(UART1_BASE))
                {

                    uint32_t  ui32Loop ;
                    char command[3];

                    for(ui32Loop = 0; ui32Loop < 3; ui32Loop++)
                    {

                        command[ui32Loop] = UARTCharGet(UART1_BASE);
                    }




                    if(command[0] == 'f' && command[1] == 'w' && command[2] == 'd')
                    {
                        // Forward: Set GPIO_PIN_2 HIGH and GPIO_PIN_4 LOW
                        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_PIN_2); // High
                        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, 0x0);        // Low
                    }

                    else if(command[0] == 'r' && command[1] == 'e' && command[2] == 'v')
                    {
                        // Reverse: Set GPIO_PIN_2 LOW and GPIO_PIN_4 HIGH
                        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, 0x0);        // Low
                        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, GPIO_PIN_4); // High
                    }
                    else if(command[0] == 's' && command[1] == 't' && command[2] == 'p')
                    {
                        PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, false);
                        PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, false);

                        last_detected_line = NO_LINE;
                        onVal = 0;

                    }
                    else if(command[0] == 's' && command[1] == 't' && command[2] == 'r')
                    {
                        PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
                        PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);
                        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_PIN_2); // High
                        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, 0x0);

                        onVal = 1;
                        last_detected_line = NO_LINE;


                    }

                    else if(command[0] == 'l' && command[1] == 'o' && command[2] == 'w')
                    {
                       LeftDutyCycle = 25;
                       RightDutyCycle = 25;
                       PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, LeftDutyCycle * pwmMax / 100);
                       PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, RightDutyCycle * pwmMax / 100);

                    }
                    else if(command[0] == 'm' && command[1] == 'i' && command[2] == 'd')
                    {
                       LeftDutyCycle = 50;
                       RightDutyCycle = 50;
                       PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, LeftDutyCycle * pwmMax / 100);
                       PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, RightDutyCycle * pwmMax / 100);
                    }
                    else if(command[0] == 'm' && command[1] == 'a' && command[2] == 'x')
                    {
                       LeftDutyCycle = 100;
                       RightDutyCycle = 100;
                       PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, LeftDutyCycle * pwmMax / 100);
                       PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, RightDutyCycle * pwmMax / 100);
                    }




                }

}




int main(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                           SYSCTL_OSC_MAIN);

        IntMasterEnable();

        //
        // Enable the GPIO port that is used for the on-board LED.
        //
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);


        SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);



        while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0))
        {

        };


        //milestone 8
        SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);

        // Wait until the TIMER2 peripheral is ready
        while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER2))
        {
            // No-op; just waiting
        }

        // Once TIMER2 is ready, configure it
        TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC_UP);  // Use up-count mode to measure time



        SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
        GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6 | GPIO_PIN_7);
        GPIOPinConfigure(GPIO_PB6_M0PWM0);
        GPIOPinConfigure(GPIO_PB7_M0PWM1);

        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, pwmMax);
        PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

        PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, false);
        PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, false);

        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, RightDutyCycle * (float)pwmMax / 100);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, LeftDutyCycle * (float)pwmMax / 100);

        PWMGenEnable(PWM0_BASE, PWM_GEN_0);


        SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
        GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
        GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);
        ADCSequenceDisable(ADC0_BASE, 1);
        ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);



        ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH0);
        ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH0);
        ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH1);
        ADCSequenceStepConfigure(ADC0_BASE, 1, 3, ADC_CTL_CH1 | ADC_CTL_IE | ADC_CTL_END);
        ADCSequenceEnable(ADC0_BASE, 1);  //old ADC set up



        //
        // Enable the GPIO pins for the LED (PF2 & PF3).
        //

        GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
        GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
        GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);

        GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_4);
        GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_2);
        GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_3);
        //




// Use up-count mode to measure time




        volatile uint32_t ui32Loop;

        //
        // Initialize the UART.
        //

        ConfigureUART();


        ConfigureTimer50ms();

        ConfigureTimer40msB();
        //
        // We are finished.  Hang around doing nothing.
        //

        /*while(1)
        {
            //
            // Turn on the BLUE LED.
            //
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

            //
            // Delay for a bit.
            //
            SysCtlDelay(SysCtlClockGet() / 10 / 3);

            //
            // Turn off the BLUE LED.
            //
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);

            //
            // Delay for a bit.
            //
            SysCtlDelay(SysCtlClockGet() / 10 / 3);
        }*/


    BIOS_start();

    return (0);
}
