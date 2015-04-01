/* File:   moving.h
 * Authors: Jordan Argyle, Trevor Buckner, Jordan Calvert, Mikkel Unrau
 * Created on March 17, 2015, 10:31 AM\

 * Steering functions - Easy way to perform repeated driving commands.
 * Note: The motors' rotation is based on the frequency of the PWM, not the duity cycle.
 * All of these use a timer to stop the PWM after the desired angle should be reached. Thus, a
 *    timer interrupt will be needed, and is included in the interrupts section. */

#ifndef MOVING_H
#define	MOVING_H
#endif

/*-------------------------------------------------------------------------------------------------
 Move forward - this function moves us forward a specific number of degrees of motor rotation. It
    uses the stepper motor controller chips from Pololu to drive both wheels forward.
 * Called from:
-------------------------------------------------------------------------------------------------*/
void forward(int deg)
{
    printText("Go forward");
    UART1PutChar('\n');
    
    LATBbits.LATB5 = 0;  // Switch both directions to be forward
    LATBbits.LATB6 = 0;

    OC1R = 0.2 * PR2;     // Turn on OC1R, sending pulses to stepper motors
    //Wait until finished turning (1 tic per 1.8 degrees of wheel rotation)

    //256 (prescale timer 1) * 0.

    delay((int)((2.0 * deg / 460.8) * PR2), 1); // halt the rest of the program for duration of motion
    
    // Set expected distance to check against ultrasound later

    OC1R = 0;           // Turn off OC1R, which controls the steppers driving our robot
}

/*-------------------------------------------------------------------------------------------------
 Move backwards - this function moves us backwards a specific number of degrees of motor rotation.
    It uses the stepper motor controller chips from Pololu to drive both wheels backwards.
 * Called from:
-------------------------------------------------------------------------------------------------*/
void backwards(int deg)
{
    printText("Go backward");
    UART1PutChar('\n');
    
    LATBbits.LATB5 = 1;  // Switch both directions to be forward
    LATBbits.LATB6 = 1;

    OC1R = 0.2 * PR2;     // Turn on OC1R, sending pulses to stepper motors
    //Wait until finished turning (1 tic per 1.8 degrees of wheel rotation)
    delay((int)((2.0 * deg / 460.8) * PR2), 1); // halt the rest of the program for duration of motion

    OC1R = 0;           // Turn off OC1R, which controls the steppers driving our robot
}

/*-------------------------------------------------------------------------------------------------
 Turn right - this function turns us to the right a specific number of degrees of base rotation
    (not motor degrees). It uses the stepper motor controller chips from Pololu to drive the wheels
    in opposite directions, turning us.
 * Called from:
-------------------------------------------------------------------------------------------------*/
void turnRight(int deg)   //USES MOTOR DEGREES AT THE MOMENT    //green black blue red, black green blue red.  red on b2
{
    printText("Turn right");
    UART1PutChar('\n');
    
    LATBbits.LATB5 = 1;  // Switch both directions to be forward
    LATBbits.LATB6 = 0;

    OC1R = 0.2 * PR2;     // Turn on OC1R, sending pulses to stepper motors
    //Wait until finished turning (1 tic per 1.8 degrees of wheel rotation)
    delay((int)((2.0 * deg / 460.8) * PR2), 1); // halt the rest of the program for duration of motion

    OC1R = 0;           // Turn off OC1R, which controls the steppers driving our robot
}

/*-------------------------------------------------------------------------------------------------
 Turn left - this function turns us to the left a specific number of degrees of base rotation
    (not motor degrees). It uses the stepper motor controller chips from Pololu to drive the wheels
    in opposite directions, turning us.
 * Called from:
-------------------------------------------------------------------------------------------------*/
void turnLeft(int deg)    //USES MOTOR DEGREES AT THE MOMENT
{
    printText("Turn left");
    UART1PutChar('\n');
    
    
    LATBbits.LATB6 = 1;
    LATBbits.LATB5 = 0;  // Switch both directions to be forward

    OC1R = 0.2 * PR2;     // Turn on OC1R, sending pulses to stepper motors
    //Wait until finished turning (1 tic per 1.8 degrees of wheel rotation)
    delay((int)((2.0 * deg / 460.8) * PR2), 1); // halt the rest of the program for duration of motion

    OC1R = 0;           // Turn off OC1R, which controls the steppers driving our robot
}

void stopDriving()
{
    printText("Stop driving\n");

    OC1R = 0;
}

void forwardUntil()
{
    printText("Start Forward\n");

    LATBbits.LATB5 = 0;  // Switch both directions to be forward
    LATBbits.LATB6 = 0;

    OC1R = 0.2 * PR2;     // Turn on OC1R, sending pulses to stepper motors
}

void backwardUntil()
{
    printText("Start Backward\n");

    LATBbits.LATB5 = 1;  // Switch both directions to be forward
    LATBbits.LATB6 = 1;

    OC1R = 0.2 * PR2;     // Turn on OC1R, sending pulses to stepper motors
}

void turnLeftUntil()
{
    printText("Start Left\n");

    LATBbits.LATB5 = 0;  // Switch both directions to be forward
    LATBbits.LATB6 = 1;

    OC1R = 0.2 * PR2;     // Turn on OC1R, sending pulses to stepper motors
}

void turnRightUntil()
{
    printText("Start Right\n");

    LATBbits.LATB5 = 1;  // Switch both directions to be forward
    LATBbits.LATB6 = 0;

    OC1R = 0.2 * PR2;     // Turn on OC1R, sending pulses to stepper motors
}

