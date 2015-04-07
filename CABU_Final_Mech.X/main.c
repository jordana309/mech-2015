/* File:   main.c
 * Authors: Jordan Argyle, Trevor Buckner, Jordan Calvert, Mikkel Unrau
 * ME 495R: Mechatronics
 * Final Project
 * Created on February 08, 2015, 20.05
 *
 * This program includes all the code for our final mechatronics project
 *   0) Compiler Direcectives
 *      a) Includes
 *      b) Directives for coding and Global Variables
 *      c) Microprocessor configuration
 *   1) Initial configuration - config.h
 *      a) Pin configuration
 *      b) PWM configuration
 *      c) A/D configuration
 *      d) Declare useful general functions
 *      e) Interrupts
 *   2) Orientation in the first 5 seconds
 *      a) Rotate until positioning LED at "max"
 *      b) Back into the corner
 *      c) Detect contact on both touch sensors
 *      d) Contengency plan?
 *   3) Navigate to ball dispenser
 *      a) Move forward to center(ish)
 *      b) Turn rear to ball corner and back in
 *      c) Check position relative to walls with ultrasound
 *   4) Ball collection
 *      a) Flip out paddle
 *      b) Make sure that ball made it into bowl
 *      c) Make position corrections if needed
 *      d) Collect the other 5
 *   5) Navigation from ball collection to center of arena
 *      a) Measure position from walls with ultrasound
 *      b) Drive to approximate center (known number of wheel turns)
 *      c) Double-check with ultrasound
 *   6) Target Acquisition
 *      a) Spin. Measure both position from walls with ultrasound and presence of LED
 *      b) Upon aquiring target, turn to exactly face corner (known number of wheel turns)
 *      c) Double check with ultrasound.
 *   7) Firing the balls
 *      a) Once squared up, spin up the shooter
 *      b) Allow one ball into the turrent chamber.
 *      c) Land it perfectly in the target
 *   8) Repeat
 */

///////////////////////////////////////////////////////////////////////////////////////////////////
// 0) Compiler Direcectives - Gets our code ready to be compiles and selects the oscilator.
///////////////////////////////////////////////////////////////////////////////////////////////////

/*-------------------------------------------------------------------------------------------------
 0a) Includes - Include the IO libraries and microcontroller header.
-------------------------------------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <p24FJ128GA202.h>

/*-------------------------------------------------------------------------------------------------
 0b) Directives for coding - Define globals, set up a compiler variable, so we can include other
     code that is only occationally needed. ie for testing, when we would include the setup for
     bluetooth. To use this, wrap the necessary code between #ifdef VAR and #endif.
-------------------------------------------------------------------------------------------------*/
// Wraps all code that we need for testing, and allows removal when unneeded.
#define TESTING
#ifdef TESTING
/* Puts in all our debugging coded. Functions included:
 * UART1CONFIG */
#include "debug.h"
#endif

// Global Vars
// A "boolean" to let us know if we're currently doing something with the timer
volatile int processingTask = 0;
// A "boolean" to let us know if we're currently waiting for the ultrasonic to get a response
volatile int waitingUS = 0;
volatile float usonicL = 0; // Value read from the left Ultrasonic
volatile float usonicR = 0; // Value read from the right Ultrasonic

/*-------------------------------------------------------------------------------------------------
 0c) Microprocessor configuration - Set our 4 configuraion registers (Con Words).
     Each variable needs to be configured by name. Vars that we set include comments

 REFs:  29.0 ,   9.7  sections, which are
      (p356) , (p154) in the datasheet
-------------------------------------------------------------------------------------------------*/
/* Con Word 1: Register 29-1 (pg 350)
 * JTAGEN is something connected to debugging options. Disabling this opens up some pins */
_CONFIG1(JTAGEN_OFF);
//_CONFIG1(JTAGEN_ON);

/* Con Word 2: Register 29-2 (pg 352)
 * Use High-speed clock in HS mode (High speed crystal resonator) = 8-32 MHz
 * This combination allows more pins to be opened up. */
_CONFIG2(POSCMD_HS && OSCIOFCN_ON);

/* Con Word 3: none yet */
//_CONFIG3();

/* Con Word 4: none yet */
//_CONFIG4();

///////////////////////////////////////////////////////////////////////////////////////////////////
// 1) Initial configuration - Configures pins, peripherals
///////////////////////////////////////////////////////////////////////////////////////////////////

/*-------------------------------------------------------------------------------------------------
 1a) Pin configuration - Specifies which pins are used for what. A pin inventory below, more
     details in config.h.
 * Called from main() intro section.
 REF: Pin Diagram (p3), 2.0 (minimum connections, p21)

 VDD, GND minimum connection*     1 | 28  VDD - Positive Voltage In ** //Sleep and M0
         Limit switch-L           2 | 27  VSS - GND **
         Ultrasonic trigger       3 | 26  RB15 - Ball release Solenoid
         UART1                    4 | 25  AN6 - PhotoDiode Input-BR
        Only 1V output???? XXX    5 | 24  AN7 - PhotoDiode Input-FL
         Rp2 (OC3) shooter motor  6 | 23  RB12 - PWM for step output, mapped to OC1.
         Rp3 (OC2) servo rod      7 | 22  X
 VSS - GND **                     8 | 21  Limit switch-R
                            X     9 | 20  VCAP*** - Connected by 10uF Cap to GND
                            X    10 | 19  VBAT
                            X    11 | 18  X RB9 - Out, Always On to M0 setting half step NO NOW THIS IS SHOOTER OUT
                            X    12 | 17  CN22   Detect Ultrasonic-R
 VDD - Positive Voltage In **    13 | 16  CN23   Detect Ultrasonic-L
 Out, Direction to left wheel    14 | 15  Out, Direction to right wheel

    X DON'T WORK AS INPUT PINS
    O WORKS AS INPUT PINS WITH JTAG / 2NDARY OSCILLATOR OFF
*   VDD->10K resister. Branch. 100-470 resister to pin 1, 0.1uF, 20V ceramic cap to GND
**  0.1uF, 20V Ceramic Cap connects 27 to 28, 8 to 13.
*** See section 29.2 (p359) and Table 32-11 (p383) on DVR21 (CEFC), which specifies series R < 3ohm

1b-c) are in the other includes below
-------------------------------------------------------------------------------------------------*/
#include "config.h"


///////////////////////////////////////////////////////////////////////////////////////////////////
// Shooter functions - Easy way to perform repeated shooter commands.
// We have a rod that will break the IR beam, so we need functions to move the rod.
// We also need functions to turn on and off our ball shooting wheels, and to pull back the
//   solonoid that holds the balls.
///////////////////////////////////////////////////////////////////////////////////////////////////
#include "shooting.h"
/* Functions inside this header are:
   * void rodLeft(), which flips our beam-breaker to the left 45 degrees.
   * void rodRight(), which flips our beam-breaker to the right 45 degrees.
 * THE ROD FUNCTIONS REQUIRE CALIBRATION WITH OUR SERVOS.
   * TODO: SUGGESTION to change shootBall() to TurnOn/OffShooterWheels, or add stopShooting().
   * void shootBall(), which turns on PWM to a transistor allowing voltage to the shooter motors.
   * void releaseBall(), which pulls the solonoid to allow a ball to drop. */


///////////////////////////////////////////////////////////////////////////////////////////////////
// Steering functions - Easy way to perform repeated driving commands.
// Note: The motors' rotation is based on the frequency of the PWM, not the duity cycle.
// All of these use a timer to stop the PWM after the desired angle should be reached. Thus, a
//   timer interrupt will be needed, and is included in the interrupts section.
///////////////////////////////////////////////////////////////////////////////////////////////////
#include "moving.h"
/* Functions inside this header are:
   * void forward(int deg), which moves forward a given number of degrees.
   * void backwards(int deg), which moves backwards a given number of degrees.
 * Both wheels spin the same way for forward() and backward(), and the number of degrees in the
 * argument is the number of degrees that the wheels rotate.
   * void turnRight(int deg), which turns the robot to the right
   * void turnLeft(int deg), which turns the robot to the left.
 * The wheels turn in opposite directions for turning, and the number of degrees in the argument
 * is the number of degrees that the base will rotate, not the wheels.
 * THIS REQUIRES CALIBRATION WITH OUR WHEELS. */


///////////////////////////////////////////////////////////////////////////////////////////////////
// Sensor functions - Functions to ease use of sensors. Sensors on board:
// * Two limit sensors for detecting contact with wall.
//   Requires one pin each = 2 pins.
// * Two ultrasound sensors - handled with interrupts only. Initialized in 2) Orientation.
//   Requires two pins each = 4 pins.
// * Two IR sensor for detecting the IR LEDs.
//   Requires a low pass filter, op-amp, and one pin each = 2 flters, op-amps, and pins
// * An IR break-beam sensor for the balls
//   Requires one pin
///////////////////////////////////////////////////////////////////////////////////////////////////
#include "sensors.h"
/* Functions inside this header are:
   * int isLimitSwitchLPressed(), which returns 1 (pressed) or 0 (not pressed) for L limit switch.
   * int isLimitSwitchRPressed(), which returns 1 (pressed) or 0 (not pressed) for R limit switch.
   * void pulseUltra(), which triggers our ultrasonic sensors to measure distance
   * */
 // TODO: add function for IR sensors and break-beam sensor

/*-------------------------------------------------------------------------------------------------
 1d) Declare useful general functions
     There are a few functions that are used in many places. We declare those here.
-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------
 * Delay - this function idles the program for the specified amount of time, locking all other
    functionality.
 * Args: tics - number of processor cycles to wait
         lock - lock out all other functions. 0=no, 1=yes
 -------------------------------------------------------------------------------------------------*/
// TODO: Currently, both moving and pusing the Ultrasound use this function, and we need a way to
// differentiate or disallow one to happen if the other is already happening. Or just make a
// different delay function for each.
void delay(int tics, int lock)            //16 tics = 1 ms, 16000 tics = 1 sec
{
    processingTask = 1; // Allows us to know if something else is already being processed
    TMR1 = 0;           //Reset Timer 1
    PR1 = tics;         //Set Timer 1 clock
    _T1IE = 1;          //Enable Timer 1 interrupt. Disabled in interrupt
    if(lock!=0)
    {
        while(processingTask) {}    //Wait for Timer 1 to finish
    }
}

/*-------------------------------------------------------------------------------------------------
 1e) Declare Interrupts
     These are all the interrupts that run our state machine
-------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------
 Stop wheels - this is a timer interrupt that will kill the wheel rotation, stopping our bot from
    moving forward, backwards, or turning. The timer value is calcuated in each function setting
    it up.
 * Set up in: all Steering functions above
-------------------------------------------------------------------------------------------------*/
void _ISR _T1Interrupt(void)
{
    _T1IF = 0;          // Clear interrupt flag
    _T1IE = 0;          // Disable Timer 1 interrupt
    
    processingTask = 0; // Reset flag that tells us if we already have an active driving command
}

/*-------------------------------------------------------------------------------------------------
 Ultrasound data recieved - Looks at the data that we've received, and makes it useful. The US
    sensors return a PWM where the duty cycle specifies the distance, so we need to detect when
    it changes, time it, and then use that timed value before it changes again to calculate
    distance.
 * Set up in: pulseUltra()
-------------------------------------------------------------------------------------------------*/
// TODO: impliment this for each US seperately, because we need to detect the response from each
// seperately, as they will not start returning a signal at the same time, which is what this
// code assumes.
void _ISR _CNInterrupt(void)
{
    // Initialized to 0, but maintains value set each time this is called between callings.
    static int prevUSL = 0; // Previous value for left  ultrasound.
    static int prevUSR = 0; // Previous value for right ultrasound.
    _CNIF = 0;              // Clear interrupt flag (IFS1 register)

    int readUSL = _RB7;     // Get current value for left US
    int readUSR = _RB8;     // Get current value for right US

    // Just got echo from USonic.  Start timing
    if(prevUSL == 0 && prevUSR == 0)
    {
        TMR5 = 0;
        T5CONbits.TON = 1; // Turn timer on
        prevUSL = 1;
        prevUSR = 1;
    // Response from US1 finished.
    } else if(prevUSL == 1 && readUSL == 0) {
    
        int timePassed = TMR5;
        prevUSL = 0;
        usonicL = timePassed * 2.0 / 58;     // timepassed is 2 microseconds per tic.  microseconds / 58 gives distance in cm
    // Response from US2 finished.
    } else if(prevUSR == 1 && readUSR == 0) {
        int timePassed = TMR5;
        prevUSR = 0;
        //if(TMR5 > 3480)                    // IF we detect > 120 cm (the width of the arena)
        //    return;
        usonicR = timePassed * 2.0 / 58;     // timepassed is 2 microseconds per tic.  microseconds / 58 gives distance in cm
    //Both US responses have finished
    } else if(readUSL == 0 && readUSR == 0) {
        T5CONbits.TON = 0;  //Turn timer off
    }
    waitingUS = 0;
}

/*-------------------------------------------------------------------------------------------------
 Ball loaded - This interrupt detects when a ball is in the hopper and ready to go
 * Set up in: 4) Ball collection, 7) Firing the balls
-------------------------------------------------------------------------------------------------*/
// TODO: Code this.
/*void _ISR _TxInterrupt(void)
{
    // Instruct loading or firing to continue
    // Kill the timer seeing if loading fails
    // NOTE: If this doesn't happen before a timer expires, the robot will "jiggle" to try to dislodge a ball
}*/

/*-------------------------------------------------------------------------------------------------
 Ball not loading properly - This is a timer function
 * Set up in: 4) Ball collection,  7) Firing the balls
-------------------------------------------------------------------------------------------------*/
// TODO: Code this.
/*void _ISR _TxInterrupt(void)
{
    // IF collecting balls, then either the paddle failed we are positioned poorly.
    // Try swiping again. If that fails again, test position and move if needed.
    // If ultrasound tells us we're good, then "jiggle" to move ball into hopper.
    // ELSE IF firing, "jiggle" to dislodge balls into the hopper
}*/

/*-------------------------------------------------------------------------------------------------
 Target time expired - This allows our bot to know if the window for orientation or active target
    has changed. If it has, we need to reorient ourselves.
 * Set up in: 
-------------------------------------------------------------------------------------------------*/
// TODO: Code this.
/*void _ISR _TxInterrupt(void)
{
    // After the initial 5 seconds, time will be reset for 30 seconds for 1st, 2nd, 3rd goal times
    // After final goal time expires, it will command the bot to do something fun/funny (like bowing)
}*/

/*-------------------------------------------------------------------------------------------------
 Competition time expired - This allows our bot to know if the competition is over. This can also
    be wrapped into the target time expired function, counting the number of shooting rounds.
 * Set up in:
-------------------------------------------------------------------------------------------------*/
// TODO: Code this.
/*void _ISR _TxInterrupt(void)
{
    // After the initial 5 seconds, time will be reset for 30 seconds for 1st, 2nd, 3rd goal times
    // After final goal time expires, it will command the bot to do something fun/funny (like bowing)
}*/

///////////////////////////////////////////////////////////////////////////////////////////////////
// Main function - this is the primary code of the project. This calls/sets up all functions above.
///////////////////////////////////////////////////////////////////////////////////////////////////
int main()
{
    /* Intro section - 1) Initial configuration */

    int i;
    int ballsHeld = 0;

    _RCDIV = 0x0; // No postscaler for oscilator

    // Configure PWM, A/D
    PinConf();
    PWMConf();
    ADConf();
    
    #ifdef TESTING
      UART1Config();
    #endif

    /* 2) Orientation in the first 5 seconds
         *      a) Rotate until positioning LED at "max"
         *      b) Back into the corner
         *      c) Detect contact on both touch sensors
         *      d) Contengency plan? */

        /* 3) Initial navigation to ball dispenser
         *      a) Move forward to center(ish)
         *      b) Turn rear to ball corner and back in
         *      c) Check position relative to walls with ultrasound */

    delay(16000, 1);

//    startShooting();
//    while(1)
//    {
//        //startShooting();
//        releaseBall();
//        //stopShooting();
//        delay(16000, 1);     //Wait .25 secs
//    }

    /* 8) Repeat. Main program start. */
    while(1)
    {
        //pulseUltra();

        //Locate IR Beacon

        float IRBeacon1;

    //Turn left until we see an IR signal
        turnLeftUntil();

        printText("Look for IR\n");
        while(1)
        {
            IRBeacon1 = checkLIR();

            if(IRBeacon1 > 2.2) //No signal at 0.95-sh.  2.7 in middle of ring, 2.2 at opposite end
            {
                stopDriving();
                break;
            }
        }
        
    //Back into the corner
        printText("Go to corner/n");
        backwardUntil();
        while(1)
        {
            if(isLimitSwitchLPressed() && isLimitSwitchRPressed())
            {
                stopDriving();
                printText("Both switches pressed\n");
                delay(8000, 1); //Wait half a second for us to see it stopped
                break;
            }
        }
        
    //Move to center of arena
        printText("Move to center\n");
        while(1)
        {
            forward(360);
            forward(360);
            forward(360);
            forward(360);
            //forward(360);
            break;
        }

    //Turn to face Garage
        printText("Turn to garage\n");
        turnRight(360);  //Turns 90 degrees

    //Drive toward Garage
        printText("Enter garage\n");
        backwardUntil();
        while(1)
        {
            if(isLimitSwitchLPressed() && isLimitSwitchRPressed())
            {
                stopDriving();
                printText("Both switches pressed\n");
                printText("In garage corner!\n");
                delay(8000, 1); //Wait half a second for us to see it stopped
                break;
            }
        }


        ballsHeld++;
        delay(8000, 1);
        //Back in and out to get balls
        forward(180); //Back up just a smidge
        

        backwards(180);
        ballsHeld++;
        delay(8000, 1);
        forward(180);
        

        backwards(180);
        delay(8000, 1);
        ballsHeld++;
        forward(180);
        


        //Drive back to center of ring
        forward(180);
        forward(360);
        forward(360);
        forward(360);

        //Turn to face first LED
        turnLeft(180);

        //Turn left until we see an IR signal
        turnLeftUntil();

        printText("Look for IR\n");
        while(1)
        {
            IRBeacon1 = checkLIR();

            if(IRBeacon1 > 2.5) //No signal at 0.95-sh.  2.7 in middle of ring, 2.2 at opposite end
            {
                stopDriving();
                break;
            }
        }

        //Turn 180 Deg to face target
        //turnRight(360);
        //turnRight(360);

        //Release a ball and shoot it
        while(ballsHeld > 0)
        {
            startShooting();
            releaseBall();
            stopShooting();
            ballsHeld--;
        }


    //Trigger ball release with Solenoid
//        for(i = 1; i <= 6; i++) //six times
//        {
//            rodLeft();      //Flip out servo
//            rodRight();     //Flip back
//        }

        break;


//        while(1)
//        {
//            turnLeft(90);
//            printText("Look for IR");
//            UART1PutChar('\n');
//
//            float IRBeacon = checkLIR();
//
//            if(IRBeacon > 1.5) //No signal at 0.957
//            {
//                break;
//            }
//        }
//
//        //Wait until both limit switches are hit
//        while(!isLimitSwitchLPressed() || !isLimitSwitchRPressed())
//        {
//            printText("Go to corner");
//            UART1PutChar('\n');
//            if(!isLimitSwitchLPressed() && !isLimitSwitchRPressed())
//            {
//                backwards(20);  //Back up (turn wheels 20 degrees)
//            }
//            else if(isLimitSwitchLPressed())
//            {
//                backwards(20);
//                if(isLimitSwitchRPressed() && isLimitSwitchLPressed())
//                {
//                    break;
//                }
//                turnLeft(80);
//            }
//            else if(isLimitSwitchRPressed())
//            {
//                backwards(20);
//                if(isLimitSwitchRPressed() && isLimitSwitchLPressed())
//                {
//                    break;
//                }
//                turnRight(80);
//            }
//        }
    //Drive to center
        //forward(360);
        //forward(360);
        //forward(360);
        //forward(360);
        //forward(360);
//      while (1){
//        forward(360);
//        delay(16000, 1);
//        backwards(360);
//        delay(16000, 1);
//        turnLeft(360);
//        delay(16000, 1);
//        turnRight(360);
//        delay(16000, 1);
//      }


        //rodLeft();
        //rodRight();
        //shootBall();
        //releaseBall();

        /* 4) Ball collection
         *      a) Flip out paddle
         *      b) Make sure that ball made it into bowl
         *      c) Make position corrections if needed
         *      d) Collect the other 5 */

        /* 5) Navigation from ball collection to center of arena
         *      a) Measure position from walls with ultrasound
         *      b) Drive to approximate center (known number of wheel turns)
         *      c) Double-check with ultrasound */

        /* 6) Target Acquisition
         *      a) Spin. Measure both position from walls with ultrasound and presence of LED
         *      b) Upon aquiring target, turn to exactly face corner (known number of wheel turns)
         *      c) Double check with ultrasound. */

        /* 7) Firing the balls
         *      a) Once squared up, spin up the shooter
         *      b) Allow one ball into the turrent chamber.
         *      c) Land it perfectly in the target */

        /* 3) Navigate to ball dispenser
         *      a) Move forward to center(ish)
         *      b) Turn rear to ball corner and back in
         *      c) Check position relative to walls with ultrasound */
    }
    while(1)
    {}
    
    return 0;
}