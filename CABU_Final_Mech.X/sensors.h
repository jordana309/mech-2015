/* File:   sensors.h
 * Author: Jordan Argyle, Trevor Buckner, Jordan Calvert, Mikkel Unrau
 * ME 495R: Mechatronics

 * This is a header file for all the sensor functions. State machine logic will be contained
 * in main.c, but this file will group all our different sensors. We have the following sensors:

 * Two limit sensors for detecting contact with wall.
   Requires one pin each = 2 pins.
 * Two ultrasound sensors - handled with interrupts only. Initialized in 2) Orientation.
   Requires two pins each = 4 pins.
 * Two IR sensor for detecting the IR LEDs.
   Requires an op-amp and filtering, and one pin each = 2 pins
 * An IR break-beam sensor for the balls
   Requires one pin */

#ifndef SENSORS_H
  #define SENSORS_H
#endif

/*-------------------------------------------------------------------------------------------------
 Check left limit switch - this function simply returns a "boolean", telling us if the left limit
    switch is pressed. Since booleans don't exist in C, we read a value of ground as pressed, and
    anything else as open.
 * Called from:
-------------------------------------------------------------------------------------------------*/
int isLimitSwitchLPressed()
{
    int read = _RA0;

    if(read == 0)       // 0 = pressed
    {
        return 1;
        // Echo value to bluetooth
        #ifdef TESTING
          printText('LLim On');
          ('\n');
        #endif
    } else              // 1 = open
        return 0;
}

/*-------------------------------------------------------------------------------------------------
 Check right limit switch - this function simply returns a "boolean", telling us if the right limit
    switch is pressed. Since booleans don't exist in C, we read a value of ground as pressed, and
    anything else as open.
 * Called from:
-------------------------------------------------------------------------------------------------*/
int isLimitSwitchRPressed()
{
    int read = _RB10;

    if(read == 0)       // 0 = pressed
    {
        return 1;
        // Echo value to bluetooth
        #ifdef TESTING
          printText('RLim On');
          ('\n');
        #endif
    } else              // 1 = open
        return 0;
}

/*-------------------------------------------------------------------------------------------------
 Pulse Ultrasonics - Tells the ultrasonic sensors to fire pulses and figure out where we are. The
    data is returned as a PWM, where the duty cycle specifies the distance. As such, we listen for
    the response using a ChangeNotification interrupt, so we can time it out. We also want to be
    sure that we don't pulse again before it's recieved data.
 * Called from:
-------------------------------------------------------------------------------------------------*/
void pulseUltra()
{
    if(waitingUS==0)
    {
        LATAbits.LATA1 = 1;     // Send a pulse along trigger for 0.5ms
        delay(8, 1);            // TIME MAY HAVE CHANGED
        waitingUS = 1;          // Set a flag so we know we're waiting. Reset in CN notification.
        LATAbits.LATA1 = 0;     // Turn off trigger
    }
    
    // Echo value to bluetooth
    #ifdef TESTING
      printText('LUR: ');
      printFloat(usonicL);
      printText(' RUR: ');
      printFloat(usonicR);
      UART1PutChar('\n');
    #endif
}

/*-------------------------------------------------------------------------------------------------
 Check left IR - Read from the IR sensor 5 times, and average the values. This is essentially a
    digital filter to clean up the noise on the IR line. After, cycle the variables so that a
    moving average is maintained.
 * Called from:
-------------------------------------------------------------------------------------------------*/
float checkLIR()
{
    // Previous 4 values (val1-4). val5 will hold the new value.
    static float val1 = 0;
    static float val2 = 0;
    static float val3 = 0;
    static float val4 = 0;
    static float val5 = 0;

    // Pull in current value
    val5 = 3.3 * ADC1BUF6 / 4095.0;

    // Calculate average
    float avg = (val1+val2+val3+val4+val5)/5.0;

    // cycle data
    val1=val2;
    val2=val3;
    val3=val4;
    val4=val5;

    // Echo value to bluetooth
    #ifdef TESTING
      printText("LIR: ");
      printFloat(avg);
      UART1PutChar('\n');
    #endif

    return(avg);
}

/*-------------------------------------------------------------------------------------------------
 Check right IR - Read from the IR sensor 5 times, and average the values. This is essentially a
    digital filter to clean up the noise on the IR line. After, cycle the variables so that a
    moving average is maintained.
 * Called from:
-------------------------------------------------------------------------------------------------*/
float checkBIR()
{
    // Previous 4 values (val1-4). val5 will hold the new value.
    static float val1 = 0;
    static float val2 = 0;
    static float val3 = 0;
    static float val4 = 0;
    static float val5 = 0;

    // Pull in current value
    val5 = 3.3 * ADC1BUF7 / 4095.0;

    // Calculate average
    float avg = (val1+val2+val3+val4+val5)/5;

    // cycle data
    val1=val2;
    val2=val3;
    val3=val4;
    val4=val5;

    // Echo value to bluetooth
    #ifdef TESTING
      printText("BIR: ");
      printFloat(avg);
      UART1PutChar('\n');
    #endif
    return(avg);
}

/*-------------------------------------------------------------------------------------------------
 Break-beam sensors -
 * Called from:
-------------------------------------------------------------------------------------------------*/
void BBSensors()
{
    // Echo value to bluetooth
    #ifdef TESTING
      //Things
    #endif
}