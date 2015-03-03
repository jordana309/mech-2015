/* File:   main.c
 * Authors: Jordan Argyle, Trevor Buckner, Jordan Calvert, Mikkel Unrau
 * ME 495R: Mechatronics
 * Final Project
 * Created on February 08, 2015, 20.05
 *
 * This program includes all the code for our final mechatronics project
 *   0) Compiler Direcectives
 *      a) Includes
 *      b) Directives for coding
 *      c) Microprocessor configuration
 *   1) Initial configuration
 *      a) Pin configuration
 *      b) PWM configuration
 *      c) A/D configuration
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
//#include<p24F16KA301.h> // Maintain as place-holder, in case we need to downgrade
#include <p24FJ128GA202.h>

/*-------------------------------------------------------------------------------------------------
 0b) Directives for coding - Set up a compiler variable, so we can include other code that is
     only occationally needed. An example of this would be for testing, when we would include the
     setup for bluetooth. To use this, wrap the necessary code between #ifdef VAR and #endif.
-------------------------------------------------------------------------------------------------*/
#define TESTING; // Wraps all code that we need for testing, and allows removal when unneeded.

/*-------------------------------------------------------------------------------------------------
 0c) Microprocessor configuration - Set our 4 configuraion registers (Con Words).
     Each variable needs to be configured by name. Vars that we set include comments
 
 REFs: Sec. 29.0,   9.7  sections, which are
         (p356) , (p154) in the datasheet
-------------------------------------------------------------------------------------------------*/
/* Con Word 1: none yet */
//_CONFIG1();

/* Con Word 2: Oscilator selector (section 9.7 (p154) */
_CONFIG2(FNOSC_FRCPLL); // 8 MHz w/ PLL (phase-locked loop), allowing increased maximum clock speed

/* Con Word 3: none yet */
//_CONFIG3();

/* Con Word 4: none yet */
//_CONFIG4();

///////////////////////////////////////////////////////////////////////////////////////////////////
// 1) Initial configuration - Configures pins, periferals
///////////////////////////////////////////////////////////////////////////////////////////////////

/*-------------------------------------------------------------------------------------------------
 1a) Pin configuration - Specifies which pins are used for what. A pin inventory:
 * Called from main() intro section.
 REF: Pin Diagram (p3), 2.0 (minimum connections, p21)
 VDD, GND minimum connection*     1 | 28 VDD - Positive Voltage In **
                                  2 | 27 VSS - GND **
                                  3 | 26 RB15 - Out, "Sleep" on motor drivers
                                  4 | 25 RB14 - Out, Direction to left wheel
                                  5 | 24 RB13 - Out, Direction to right wheel
                                  6 | 23 RB12 - PWM for step output, mapped to OC1.
                                  7 | 22
 VSS - GND **                     8 | 21
                                  9 | 20 VCAP*** - Connected by 10uF Cap to GND
                                 10 | 19
                                 11 | 18
                                 12 | 17
 VDD - Positive Voltage In **    13 | 16
                                 14 | 15

* VDD->10K resister. Branch. 100-470 resister to pin 1, 0.1uF, 20V ceramic cap to GND
** 0.1uF, 20V Ceramic Cap connects 27 to 28, 8 to 13.
*** See section 29.2 (p359) and Table 32-11 (p383) on DVR21 (CEFC), which specifies series R < 3ohm

 REF:     ANSA/B   , Sec. 11.2  Registers, which are
      11-1/2 (p169),   (p168)   in the data sheet
-------------------------------------------------------------------------------------------------*/
// TODO: Put pin configuration here
void PinConf()
{
    TRISA = 0x0;    // output for all port A (0000), 4 A bits
    TRISB = 0x0000; // output for all port B (0000 0000 0000 0000), 16 B bits
    ANSA = 0x0; // turn off analog input on all pins, port A
    ANSB = 0x0; // turn off analog input on all pins, port B
    // Pin 23: OC1, (RB12), to be used for PWM to motors
    // TODO: Configure OC1 on pin 23
    // Pin 24: Digital Out, direction of right wheel
    // Pin 25: Digital Out, direction of left wheel
    // Pin 26: Digital Out, "sleep" mode on motor driver chips
}


/*-------------------------------------------------------------------------------------------------
 1b) PWM Configuration - Sets up our PWM output
 Called from main() intro section.

 REFs:   OSxCON1  ,   OSxCON2  ,   OCxR,  OCxRS ,    TxCON        Registers, which are
       15-1 (p216), 15-2 (p218), 15.3.2,3 (p213), 13-1 (p202)     in the data sheet
-------------------------------------------------------------------------------------------------*/
void PWMConf()
{
    // Configure OC1 (For Pin 23--RB12)
    // TODO: Map OC1 to Pin 23
    OC1CON1 = 0;    // Clear OC2 configuration bits
    OC1CON2 = 0;    // Clear OC2 configuration bits
    OC1CON1bits.OCTSEL = 0b000;     // Set it to use timer 2. Sets base for period.
    OC1CON2bits.SYNCSEL = 0b01100;  // Set sync select to timer 2. Sets comparisons and the "beat".
    OC1CON1bits.OCM = 0b110;        // Edge-Aligned PWM mode
    
    // PWM period = [Value+1]*Tcy*Prescaler value
    // On time (duty cycle) in ticks of the period timer. I'm using T2 as my ticker, so this is
    // ticks of PR2. Duity cycle is OC1R/PR2.
    OC1R = 2000;

    // TODO: Set up the timer better
    // Set up the timer
    T2CONbits.TCS = 0; // Internal clock
    T2CONbits.TCKPS = 0; // Don't prescale100
    TMR2 = 0; // Timer set to 0
    T2CONbits.TON = 1; // Timer is on
    PR2 =  3999; // Timer period
}

/*-------------------------------------------------------------------------------------------------
 1a) A/D Configuration Function - Configures A/D to read from XX channels in auto conversion mode.

 REFs:         AD1CON1,2,3,5         ,    AD1CHS  , AD1CSSL, AD1CSSH      Registers, which are
     24-1,2,3,5 (p316, 318, 320, 322), 24-6 (p323),   24-9 , 10 (p326)    in the data sheet.
-------------------------------------------------------------------------------------------------*/
void ADConf(void)
{
    // TODO: Configure A/D, and choose pins needed. How to do this will need to be looked at.
    //Set up A/D 1, reading on pins XX.

    // First, choose pins using AD1CHS register (22-5)

    _CH0NA = 0b000;     // Choose GND (pin 20) as negative input
    //_CH0SA = 0b01100; // Choose AN12 (pin 15) as positive input. Less reliable
                        // than using AD1CSSL/H, so we don't use it

    // AD1CON1 register (22-1)
    _ADON = 1;      // AD1CON1<15> -- A/D on?
                    // 1=On
    _ADSIDL = 0;    // AD1CON1<13> -- A/D stops while in idle mode?
                    // 0=yes
    _MODE12 = 1;    // AD1CON1<10> -- 12-bit or 10-bit?
                    // 1=12
    _FORM = 0b10;   // AD1CON1<9:8> -- Output format
                    // 00=Abs decimal, unsigned
                    // 10=Abs fractional, unsigned
    _SSRC = 0b0111; // AD1CON1<7:4> -- Sample clock source select
                    // 0111=Auto conversion, internal counter
    _ASAM = 1;    // AD1CON1<2> -- When to sample
                    // 1=Continuous auto sampling

    // AD1CSSL/H registers (22-9 and 22-8)
    AD1CSSL = 0;    // AD1CSSL<15:0> -- Select lower channels to scan
                    // 0=all off, since we'll turn on the ones we want individually
    AD1CSSH = 0;    // AD1CSSH<15:0> -- Select upper channels to scan
                    // 0=all off; we'll turn on the ones we want individually
    //_CSS12 = 1;     // Turn on AN12 (Pin 15) to sample

    // AD1CON2 register (22-2)
    _PVCFG = 0;         // AD1CON2<15:14> -- Set positive voltage reference
                        // 0=Use VDD as positive ref voltage
    //_NVCFG = 0;         // AD1CON2<13> -- Set negative voltage reference
                        // 0=Use VSS as negative ref voltage
    _BUFREGEN = 1;      // AD1CON2<11> -- A/D buffer register enable?
                        // 1=enabled. Results stored using channel indexed
                        // mode -- AN1 result is stored in ADC1BUF1, AN2 result
                        // is stored in ADC1BUF2, etc.
    _CSCNA = 1;         // AD1CON2<10> -- Scan inputs?
                        // 1=Scans inputs specified in AD1CSSx registers instead
                        // of using channels specified by CH0SA bits in AD1CHS
    _ALTS = 0;          // AD1CON2<0> -- Alternate input sample
                        // 0=Sample MUXA only (*not from MUXB)
    _SMPI = 0b00010;    // AD1CON2<6:2> -- Sample rate interrupt select
                        // 00001=Interrupts at the conversion for every 2 samples

    // AD1CON3 register (22-3)
    _ADRC = 0;          // AD1CON3<15> -- Clock source selection
                        // 0=Use system clock
    _SAMC = 0b00001;    // AD1CON3<12:8> -- Auto-sample time select
                        // 00001=Auto sample every A/D period 1*TAD
    _ADCS = 0b00111111; // AD1CON3<7:0> -- A/D Conversion clock select
                        // 00111111=A/D period TAD = 64*TCY
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Steering functions - Easy way to perform repeated driving commands.
// Note: The motors' rotation is based on the frequency of the PWM, not the duity cycle.
// All of these use a timer to stop the PWM after the desired angle should be reached. Thus, a
//   timer interrupt will be needed, and is included in the interrupts section.
///////////////////////////////////////////////////////////////////////////////////////////////////

/*-------------------------------------------------------------------------------------------------
 Move forward - this function moves us forward a specific number of degrees of motor rotation. It
    uses the stepper motor controller chips from Pololu to drive both wheels forward.
 * Called from:
-------------------------------------------------------------------------------------------------*/
void forward(int deg)
{
    // Switch both directions to be forward
    // Calculate how long to turn wheels
    // Turn on OC1R
    // Begin timer that will let us know when to stop turning
    // Set expected distance to check against ultrasound later
}

/*-------------------------------------------------------------------------------------------------
 Move backwards - this function moves us backwards a specific number of degrees of motor rotation.
    It uses the stepper motor controller chips from Pololu to drive both wheels backwards.
 * Called from:
-------------------------------------------------------------------------------------------------*/
void backwards(int deg)
{
    // Switch both directions to be backwards
    // Calculate how long to turn wheels
    // Turn on OC1R
    // Begin timer that will let us know when to stop turning
    // Set expected distance to check against ultrasound later
}

/*-------------------------------------------------------------------------------------------------
 Turn right - this function turns us to the right a specific number of degrees of base rotation
    (not motor degrees). It uses the stepper motor controller chips from Pololu to drive the wheels
    in opposite directions, turning us.
 * Called from:
-------------------------------------------------------------------------------------------------*/
void turnRight(int deg)
{
    // Switch directions to have one forward, one backwards
    // Calculate how long to turn wheels
    // Turn on OC1R
    // Begin timer that will let us know when to stop turning
    // Set expected distance to check against ultrasound later
}

/*-------------------------------------------------------------------------------------------------
 Turn left - this function turns us to the left a specific number of degrees of base rotation
    (not motor degrees). It uses the stepper motor controller chips from Pololu to drive the wheels
    in opposite directions, turning us.
 * Called from:
-------------------------------------------------------------------------------------------------*/
void turnLeft(int deg)
{
    // Switch directions to have one forward, one backwards
    // Calculate how long to turn wheels
    // Turn on OC1R
    // Begin timer that will let us know when to stop turning
    // Set expected distance to check against ultrasound later
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Sensor functions - Any function to ease use of sensors. Sensors on board:
// * Two touch sensors
// * Two ultrasound sensors - handled with interrupts only. Initialized in 2) Orientation.
// *
///////////////////////////////////////////////////////////////////////////////////////////////////

/*-------------------------------------------------------------------------------------------------
 Check position - fires off an ultrasound ping to check where we are. An interrupt will interpret
     the returned data.
 * Called from:
-------------------------------------------------------------------------------------------------*/
void posCheck()
{
    // Check the position using ultrasound. This will send out a "ping".
    // Turn on interrupt to interpret the data sent back
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Interrupts - Keep them all together here
///////////////////////////////////////////////////////////////////////////////////////////////////

/*-------------------------------------------------------------------------------------------------
 Touch Sensor 1 fired - This interrupt detects when touch sensor 1 (on left side of bot) makes
    contact.
 * Set up in: 2) Orientation
-------------------------------------------------------------------------------------------------*/
// TODO: Code this.
/*void _ISR _TxInterrupt(void)
{
    // Handle it.
}*/

/*-------------------------------------------------------------------------------------------------
 Touch Sensor 2 fired - This interrupt detects when touch sensor 1 (on right side of bot) makes
    contact.
 * Set up in: 2) Orientation
-------------------------------------------------------------------------------------------------*/
// TODO: Code this.
/*void _ISR _TxInterrupt(void)
{
    // Handle it.
}*/

/*-------------------------------------------------------------------------------------------------
 Ultrasound data recieved - Looks at the data that we've received, and makes it useful.
 * Set up in: posCheck()
-------------------------------------------------------------------------------------------------*/
// TODO: Code this.
/*void _ISR _TxInterrupt(void)
{
    // Average several readings together to eliminate noise
    // After several have been averaged, compare this value to where we epect to be
    // Figure out if any further driving needs to take place
    // If so, send the apropriate commands to the wheels
    // If not, instruct the program to continue to the next phase
}*/

/*-------------------------------------------------------------------------------------------------
 Stop wheels - this is a timer interrupt that will kill the wheel rotation, stopping our bot from
    moving forward, backwards, or turning. The timer value is calcuated in each function setting
    it up.
 * Set up in: all Steering functions above
-------------------------------------------------------------------------------------------------*/
void _ISR _TxInterrupt(void)
{
    // Turn off OC1R
    // Check with ultrasound to see if we made it where we expected
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
 * Set up in: 2) Initial Orientation, then in this interrupt
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

    _RCDIV = 0x0; // No postscaler for oscilator

    // Configure PWM, A/D
    PinConf();
    PWMConf();
    ADConf();


    /* 2) Orientation in the first 5 seconds
         *      a) Rotate until positioning LED at "max"
         *      b) Back into the corner
         *      c) Detect contact on both touch sensors
         *      d) Contengency plan? */

        /* 3) Initial navigation to ball dispenser
         *      a) Move forward to center(ish)
         *      b) Turn rear to ball corner and back in
         *      c) Check position relative to walls with ultrasound */



    /* 8) Repeat. Main program start. */
    while(1)
    {
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

    return 0;
}

