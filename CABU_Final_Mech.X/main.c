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
 *      0) Global variables
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
//#include <PIC24F_plib.h>


//beginnning of free code

void UART1Config()
{
U1MODE = 0;
U1BRG = 25; //9600 bps at at 2e6 Fcy
U1STAbits.UTXISEL0 = 0;
U1STAbits.UTXISEL1 = 1;
U1STAbits.URXISEL = 0;
U1MODEbits.UARTEN = 1; //enables the uart
U1STAbits.UTXEN = 1; //setting this bit per the data sheet
IFS0bits.U1RXIF = 0;
}

char UART1GetChar()
{
   while(IFS0bits.U1RXIF == 0); //wait for buffer to receive
   IFS0bits.U1RXIF = 0; //reset interrupt flag
   return U1RXREG;
}
void UART1PutChar(char Ch)
{
    while(U1STAbits.UTXBF == 1); //wait for buffer to be empty
    U1TXREG = Ch;
}

// Useful string transmission functions

void printText(char text[])
{
    int i = 0;
    while(text[i] != '\0')
    {
        UART1PutChar(text[i]);
        i++;
    }
    UART1PutChar(' ');
}

void printFloat(float data)
{
    char floatString[10];
    int i = 0;
    sprintf(floatString,"%f",data);
    while(floatString[i] != '\0')
    {
        UART1PutChar(floatString[i]);
        i++;
    }
    UART1PutChar(' ');
}

void printInt(int data)
{
    char intString[10];
    int i = 0;
    sprintf(intString,"%d",data);
    while(intString[i] != '\0')
    {
        UART1PutChar(intString[i]);
        i++;
    }
    UART1PutChar(' ');
}

//end of free code

/*-------------------------------------------------------------------------------------------------
 0b) Directives for coding - Set up a compiler variable, so we can include other code that is
     only occationally needed. An example of this would be for testing, when we would include the
     setup for bluetooth. To use this, wrap the necessary code between #ifdef VAR and #endif.
-------------------------------------------------------------------------------------------------*/
// Wraps all code that we need for testing, and allows removal when unneeded.
#define TESTING 
volatile int processingTask = 0;
volatile float usonic1 = 0;
volatile float usonic2 = 0;
volatile int limitSwitch1 = 0;
volatile int limitSwitch2 = 0;

/*-------------------------------------------------------------------------------------------------
 0c) Microprocessor configuration - Set our 4 configuraion registers (Con Words).
     Each variable needs to be configured by name. Vars that we set include comments
 
 REFs: Sec. 29.0,   9.7  sections, which are
         (p356) , (p154) in the datasheet
-------------------------------------------------------------------------------------------------*/
/* Con Word 1: none yet */
//_CONFIG1();
_CONFIG1(JTAGEN_OFF);
//_CONFIG1(JTAGEN_ON);



/* Con Word 2: Oscilator selector (section 9.7 (p154) */
//_CONFIG2(FNOSC_FRCPLL); // 8 MHz w/ PLL (phase-locked loop), allowing increased maximum clock speed
_CONFIG2(POSCMD_HS && OSCIOFCN_ON);


/* Con Word 3: none yet */
//_CONFIG3();

/* Con Word 4: none yet */
//_CONFIG4();

///////////////////////////////////////////////////////////////////////////////////////////////////
// 1) Initial configuration - Configures pins, peripherals
///////////////////////////////////////////////////////////////////////////////////////////////////




/*-------------------------------------------------------------------------------------------------
 1a) Pin configuration - Specifies which pins are used for what. A pin inventory:
 * Called from main() intro section.
 REF: Pin Diagram (p3), 2.0 (minimum connections, p21)
 VDD, GND minimum connection*     1 | 28 VDD - Positive Voltage In **
                 Limit switch-L   2 | 27 VSS - GND **
             Ultrasonic trigger   3 | 26 RB15 - Out, "Sleep" on motor drivers
                    UART1         4 | 25 RB14 - Out, Direction to left wheel
           Ball release solenoid  5 | 24 RB13 - Out, Direction to right wheel
         Rp2 (OC3) shooter motor  6 | 23 RB12 - PWM for step output, mapped to OC1.
         Rp3 (OC2) servo rod      7 | 22 X
 VSS - GND **                     8 | 21 Limit switch-R
                            X     9 | 20 VCAP*** - Connected by 10uF Cap to GND
                            X    10 | 19
                            X    11 | 18 X RB9 - Out, Always Off to M0 setting half step
                            X    12 | 17 CN22   Detect Ultrasonic2
 VDD - Positive Voltage In **    13 | 16 CN23   Detect Ultrasonic1
             PhotoDiode Input 1  14 | 15 PhotoDiode Input 2

 *
 * X DUNT WURK
 *
 * O WORKS WITH JTAG / 2NDARY OSCILLATOR OFF
* VDD->10K resister. Branch. 100-470 resister to pin 1, 0.1uF, 20V ceramic cap to GND
** 0.1uF, 20V Ceramic Cap connects 27 to 28, 8 to 13.
*** See section 29.2 (p359) and Table 32-11 (p383) on DVR21 (CEFC), which specifies series R < 3ohm

 REF:     ANSA/B   , Sec. 11.2  Registers, which are
      11-1/2 (p169),   (p168)   in the data sheet
-------------------------------------------------------------------------------------------------*/
// TODO: Put pin configuration here
void PinConf()
{
    OSCCONbits.SOSCEN = 0;
    //OSCCONbits.
    
    TRISA = 0x0;    // output for all port A (0000), 4 A bits
    TRISB = 0x0000; // output for all port B (0000 0000 0000 0000), 16 B bits
    ANSA = 0x0; // turn off analog input on all pins, port A
    ANSB = 0x0000; // turn off analog input on all pins, port B

    TRISBbits.TRISB7 = 1;   //Set Pin 16 to input (CN23) for UltraSonic1
    TRISBbits.TRISB8 = 1;   //Set Pin 17 to input (CN22) for UltraSonic2
    TRISAbits.TRISA0 = 1;   //Set Pin 2 to input (RA0) for LimitSwitch1
    TRISBbits.TRISB10 = 1;  //Set Pin 21 to input (RB10) for LimitSwitch2

    // Pin 23: OC1, (RP12), to be used for PWM to motors
    // TODO: Configure OC1 on pin 23
    // Pin 24: Digital Out, direction of right wheel (RB13)
    // Pin 25: Digital Out, direction of left wheel  (RB14)
    // Pin 26: Digital Out, "sleep" mode on motor driver chips (RB15)

    _RP12R = 13;                //Assign RP12 (pin 23) to function 13 (OC1)
    _RP0R = 3;                  //Assign RP0 (pin 4) to function 3 (UTX1)
    _RP2R = 15;                 //Assign RP2 (pin 6) to function 15 (OC3)
    _RP3R = 14;                 //Assign RP3 (pin 21) to function 14 (OC2)

    LATBbits.LATB13 = 1;        //Initialize Wheel2 Dir to 0
    LATBbits.LATB14 = 1;        //Initialize Wheel1 Dir to 0
    LATBbits.LATB15 = 1;        //Initialize Sleep to 1 (turns off sleep mode)
    LATBbits.LATB9 = 1;         //Set Pin 22 to on -> send this to M0 for half step (if left unconnected it stalls)

    LATBbits.LATB1 = 1;         //Set RB1 (Pin 5) to ball release solenoid

    // Configure CN interrupt
    _CN23IE = 1; // Enable CN on pin 16 (CNEN1 register)
    _CN23PUE = 0; // Disable pull-up resistor (CNPU1 register)
    _CNIP = 6; // Set CN interrupt priority (IPC4 register)
    _CNIF = 0; // Clear interrupt flag (IFS1 register)
    _CNIE = 1; // Enable CN interrupts (IEC1 register)

    // Configure CN interrupt
    _CN22IE = 1; // Enable CN on pin 17 (CNEN1 register)
    _CN22PUE = 0; // Disable pull-up resistor (CNPU1 register)
    _CNIP = 6; // Set CN interrupt priority (IPC4 register)
    _CNIF = 0; // Clear interrupt flag (IFS1 register)
    _CNIE = 1; // Enable CN interrupts (IEC1 register)
}


/*-------------------------------------------------------------------------------------------------
 1b) PWM Configuration - Sets up our PWM output
 Called from main() intro section.

 REFs:   OSxCON1  ,   OSxCON2  ,   OCxR,  OCxRS ,    TxCON        Registers, which are
       15-1 (p216), 15-2 (p218), 15.3.2,3 (p213), 13-1 (p202)     in the data sheet
-------------------------------------------------------------------------------------------------*/
void PWMConf()
{
//==== Configure OC1 (For Pin 23--RB12) for stepper motors
    OC1CON1 = 0;    // Clear OC1 configuration bits
    OC1CON2 = 0;    // Clear OC1 configuration bits
    OC1CON1bits.OCTSEL = 0b000;     // Set it to use timer 2. Sets base for period.
    OC1CON2bits.SYNCSEL = 0b01100;  // Set sync select to timer 2. Sets comparisons and the "beat".
    OC1CON1bits.OCM = 0b110;        // Edge-Aligned PWM mode
    
    // PWM period = [Value+1]*Tcy*Prescaler value
    // On time (duty cycle) in ticks of the period timer. I'm using T2 as my ticker, so this is
    // ticks of PR2. Duity cycle is OC1R/PR2.
    OC1R = 0;       //Begin with 0 duty cycle (no steps sent to stepper motors)

  // Set up Timer 2
    T2CONbits.TCS = 0; // Internal clock
    T2CONbits.TCKPS = 0; // Don't prescale100
    TMR2 = 0; // Timer set to 0
    T2CONbits.TON = 1; // Timer is on
    PR2 =  3999; // Timer period
//====

//==== Set up OC3 (For Pin 22--RP11) to turn the shooter motors on with a MOSFET
    OC3CON1 = 0;    // Clear OC3 configuration bits
    OC3CON2 = 0;    // Clear OC3 configuration bits
    OC3CON1bits.OCTSEL = 0b001;     // Set it to use timer 3. Sets base for period.
    OC3CON2bits.SYNCSEL = 0b01101;  // Set sync select to timer 3. Sets comparisons and the "beat".
    OC3CON1bits.OCM = 0b110;        // Edge-Aligned PWM mode

    OC3R = 2000;

  // Set up Timer 3
    T3CONbits.TCS = 0; // Internal clock
    T3CONbits.TCKPS = 0; // Don't prescale100
    TMR3 = 0; // Timer set to 0
    T3CONbits.TON = 1; // Timer is on
    PR3 = 4999; // Timer period
//====

//==== Set up OC2 to move servo for ball collection
    OC2CON1 = 0;    // Clear OC3 configuration bits
    OC2CON2 = 0;    // Clear OC3 configuration bits
    OC2CON1bits.OCTSEL = 0b010;     // Set it to use timer 3. Sets base for period.
    OC2CON2bits.SYNCSEL = 0b01110;  // Set sync select to timer 3. Sets comparisons and the "beat".
    OC2CON1bits.OCM = 0b110;        // Edge-Aligned PWM mode

    OC2R = 10000;

  // Set up Timer 4
    T4CONbits.TCS = 0; // Internal clock
    T4CONbits.TCKPS = 0; // Don't prescale100
    TMR4 = 0; // Timer set to 0
    T4CONbits.TON = 1; // Timer is on
    PR4 =  19999; // Timer period
//====

    //Set up timer for determining how long to perform operations
    T1CONbits.TON = 1;      //Don't turn Timer 1 on until we need it
    T1CONbits.TCKPS = 0b11;    //Prescale by 256
    T1CONbits.TCS = 0;      //Use internal clock

    _T1IE = 0;          //Enable T1 Interrupt
    _T1IF = 0;          //Disable T1 Interrupt Flag


    // Set up Timer 5
    T5CONbits.TCS = 0; // Internal clock
    T5CONbits.TCKPS = 0b01; // Prescale by 8
    TMR5 = 0; // Timer set to 0
    T5CONbits.TON = 0; // Timer is off
    PR5 =  65535; // Timer period


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

    //_CH0NA = 0b000;     // Choose GND (pin 20) as negative input
    //_CH0SA = 0b01100; // Choose AN12 (pin 15) as positive input. Less reliable
                        // than using AD1CSSL/H, so we don't use it

    // AD1CON1 register (22-1)
    //_ADON = 1;      // AD1CON1<15> -- A/D on?
                    // 1=On
    //_ADSIDL = 0;    // AD1CON1<13> -- A/D stops while in idle mode?
                    // 0=yes
    //_MODE12 = 1;    // AD1CON1<10> -- 12-bit or 10-bit?
                    // 1=12
    //_FORM = 0b10;   // AD1CON1<9:8> -- Output format
                    // 00=Abs decimal, unsigned
                    // 10=Abs fractional, unsigned
    //_SSRC = 0b0111; // AD1CON1<7:4> -- Sample clock source select
                    // 0111=Auto conversion, internal counter
    //_ASAM = 1;    // AD1CON1<2> -- When to sample
                    // 1=Continuous auto sampling

    // AD1CSSL/H registers (22-9 and 22-8)
    //AD1CSSL = 0;    // AD1CSSL<15:0> -- Select lower channels to scan
                    // 0=all off, since we'll turn on the ones we want individually
    //AD1CSSH = 0;    // AD1CSSH<15:0> -- Select upper channels to scan
                    // 0=all off; we'll turn on the ones we want individually
    //_CSS12 = 1;     // Turn on AN12 (Pin 15) to sample

    // AD1CON2 register (22-2)
    //_PVCFG = 0;         // AD1CON2<15:14> -- Set positive voltage reference
                        // 0=Use VDD as positive ref voltage
    //_NVCFG = 0;         // AD1CON2<13> -- Set negative voltage reference
                        // 0=Use VSS as negative ref voltage
    //_BUFREGEN = 1;      // AD1CON2<11> -- A/D buffer register enable?
                        // 1=enabled. Results stored using channel indexed
                        // mode -- AN1 result is stored in ADC1BUF1, AN2 result
                        // is stored in ADC1BUF2, etc.
    //_CSCNA = 1;         // AD1CON2<10> -- Scan inputs?
                        // 1=Scans inputs specified in AD1CSSx registers instead
                        // of using channels specified by CH0SA bits in AD1CHS
    //_ALTS = 0;          // AD1CON2<0> -- Alternate input sample
                        // 0=Sample MUXA only (*not from MUXB)
    //_SMPI = 0b00010;    // AD1CON2<6:2> -- Sample rate interrupt select
                        // 00001=Interrupts at the conversion for every 2 samples

    // AD1CON3 register (22-3)
    //_ADRC = 0;          // AD1CON3<15> -- Clock source selection
                        // 0=Use system clock
    //_SAMC = 0b00001;    // AD1CON3<12:8> -- Auto-sample time select
                        // 00001=Auto sample every A/D period 1*TAD
    //_ADCS = 0b00111111; // AD1CON3<7:0> -- A/D Conversion clock select
                        // 00111111=A/D period TAD = 64*TCY
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Steering functions - Easy way to perform repeated driving commands.
// Note: The motors' rotation is based on the frequency of the PWM, not the duity cycle.
// All of these use a timer to stop the PWM after the desired angle should be reached. Thus, a
//   timer interrupt will be needed, and is included in the interrupts section.
///////////////////////////////////////////////////////////////////////////////////////////////////

void rodLeft()
{
    processingTask = 1;
    OC2R = 7700;    //Set pulse width to 1950 microSec (45 deg left) //MAY HAVE CHANGED BY x4
    delay(10000);   //Wait for 0.625 seconds
}

void rodRight()
{
    processingTask = 1;
    OC2R = 3900;    //Set pulse width to 975 microSec (45 deg right) //MAY HAVE CHANGED BY x4
    delay(10000);   //Wait for 0.625 seconds
}

void shootBall()
{
    OC3R = 2000;    //Pulse shooter motors to launch balls
    delay(20000);   //Wait for 1.25 seconds
    OC3R = 0;       //Turn off shooter motors
}

void releaseBall()
{
    LATBbits.LATB1 = 1;     //Open solenoid to release ball
    delay(20000);           //Wait for 1.25 seconds
    LATBbits.LATB1 = 0;     //Close solenoid
}

void delay(int tics)            //16 tics = 1 ms, 16000 tics = 1 sec
{
    processingTask = 1;
    TMR1 = 0;                   //Reset Timer 1
    PR1 = tics;                 //Set Timer 1 clock
    _T1IE = 1;                  //Enable Timer 1 interrupt
    while(processingTask) {}    //Wait for Timer 1 to finish
    _T1IE = 0;                  //Disable Timer 1 interrupt
}

void pulseUltra()
{
    LATAbits.LATA1 = 1;
    delay(8);               //Send pulse for 0.5 microsec   //TIME MAY HAVE CHANGED
    LATAbits.LATA1 = 0;
    delay(8000);            //Wait 50 millisec for echo     //TIME MAY HAVE CHANGED
                            //Also, this delay may not be needed since the echo is
                            //handled in the CN interrupt.  However, we don't want
                            //to send another pulse too soon without giving the first
                            //echo a chance to come back or we will get a continuous
                            //response
}

int isLimitSwitchLPressed()
{
    int read = _RA0;
    if(read == 0)       //0 = pressed
        return 1;
    else                //1 = open
        return 0;
}

int isLimitSwitchRPressed()
{
    int read = _RB10;
    if(read == 0)       //0 = pressed
        return 1;
    else                //1 = open
        return 0;
}

/*-------------------------------------------------------------------------------------------------
 Move forward - this function moves us forward a specific number of degrees of motor rotation. It
    uses the stepper motor controller chips from Pololu to drive both wheels forward.
 * Called from:
-------------------------------------------------------------------------------------------------*/
void forward(int deg)
{ 
    LATBbits.LATB13 = 1;            // Switch both directions to be forward
    LATBbits.LATB14 = 1;

    OC1R = 0.2 * PR2;               // Turn on OC1R, sending pulses to stepper motors
    delay((2.0 * deg)/460.8 * PR2); //Wait until finished turning (1 tic per 1.8 degrees of wheel rotation)

    // Set expected distance to check against ultrasound later
}

/*-------------------------------------------------------------------------------------------------
 Move backwards - this function moves us backwards a specific number of degrees of motor rotation.
    It uses the stepper motor controller chips from Pololu to drive both wheels backwards.
 * Called from:
-------------------------------------------------------------------------------------------------*/
void backwards(int deg)
{
    LATBbits.LATB13 = 0;            // Switch both directions to be forward
    LATBbits.LATB14 = 0;

    OC1R = 0.2 * PR2;               // Turn on OC1R, sending pulses to stepper motors
    delay((2.0 * deg)/460.8 * PR2); //Wait until finished turning (1 tic per 1.8 degrees of wheel rotation)
}

/*-------------------------------------------------------------------------------------------------
 Turn right - this function turns us to the right a specific number of degrees of base rotation
    (not motor degrees). It uses the stepper motor controller chips from Pololu to drive the wheels
    in opposite directions, turning us.
 * Called from:
-------------------------------------------------------------------------------------------------*/
void turnRight(int deg)     //USES MOTOR DEGREES AT THE MOMENT
{
    LATBbits.LATB13 = 1;            // Switch both directions to be forward
    LATBbits.LATB14 = 0;

    OC1R = 0.2 * PR2;               // Turn on OC1R, sending pulses to stepper motors
    delay((2.0 * deg)/460.8 * PR2); //Wait until finished turning (1 tic per 1.8 degrees of wheel rotation)
}

/*-------------------------------------------------------------------------------------------------
 Turn left - this function turns us to the left a specific number of degrees of base rotation
    (not motor degrees). It uses the stepper motor controller chips from Pololu to drive the wheels
    in opposite directions, turning us.
 * Called from:
-------------------------------------------------------------------------------------------------*/
void turnLeft(int deg)      //USES MOTOR DEGREES AT THE MOMENT
{
    LATBbits.LATB13 = 0;            // Switch both directions to be forward
    LATBbits.LATB14 = 1;

    OC1R = 0.2 * PR2;               // Turn on OC1R, sending pulses to stepper motors
    delay((2.0 * deg)/460.8 * PR2); //Wait until finished turning (1 tic per 1.8 degrees of wheel rotation)
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
void _ISR _T1Interrupt(void)
{
    _T1IF = 0;          //Clear interrupt flag
    //T1CONbits.TON = 0;  //And turn off the timer now
    // Turn off OC1R
    OC1R = 0;
    processingTask = 0;
    // Check with ultrasound to see if we made it where we expected
}

void _ISR _CNInterrupt(void)
{
    static int prevUS1 = 0;
    static int prevUS2 = 0;
    _CNIF = 0; // Clear interrupt flag (IFS1 register)

    int readUS1 = _RB7;
    int readUS2 = _RB8;

    if(prevUS1 == 0 && prevUS2 == 0) //Just got echo from USonic.  Start timing
    {
        TMR5 = 0;
        T5CONbits.TON = 1; // Turn timer on
        prevUS1 = 1;
        prevUS2 = 1;
    }

    else if(prevUS1 == 1 && readUS1 == 0)    //Response from US1 finished.
    {
        int timePassed = TMR5;
        prevUS1 = 0;
        usonic1 = timePassed * 2.0 / 58;                //timepassed is 2 microseconds per tic.  microseconds / 58 gives distance in cm
    }

    else if(prevUS2 == 1 && readUS2 == 0)    //Response from US2 finished.
    {
        int timePassed = TMR5;
        prevUS2 = 0;
        //if(TMR5 > 3480)      //IF we detect > 120 cm (the width of the arena)
        //    return;
        usonic2 = timePassed * 2.0 / 58;                //timepassed is 2 microseconds per tic.  microseconds / 58 gives distance in cm
    }

    else if(readUS1 == 0 && readUS2 == 0)    //Both US responses have finished
    {
        T5CONbits.TON = 0;  //Turn timer off
    }
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
    UART1Config();

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
        printFloat(usonic1);
        UART1PutChar(' ');
        printFloat(usonic2);
        UART1PutChar(' ');
        if(isLimitSwitchLPressed())
        {
            UART1PutChar('O');
            UART1PutChar('N');
            UART1PutChar('1');
            UART1PutChar(' ');
        }

        if(isLimitSwitchRPressed())
        {
            UART1PutChar('O');
            UART1PutChar('N');
            UART1PutChar('2');
            UART1PutChar(' ');
        }
        UART1PutChar('\n');
        pulseUltra();

        //turnLeft()

    //Wait until both limit switches are hit
        while(!isLimitSwitchLPressed() || !isLimitSwitchRPressed())
        {
            if(!isLimitSwitchLPressed() && !isLimitSwitchRPressed())
            {
                backwards(20);  //Back up (turn wheels 20 degrees)
            }
            else if(isLimitSwitchLPressed())
            {
                turnLeft(80);
            }
            else if(isLimitSwitchRPressed())
            {
                turnRight(80);
            }

        }
    //Drive to center
        forward(360);
        forward(360);
        forward(360);
        forward(360);
        forward(360);

        /*forward(360);
        backwards(360);*/
 


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

    return 0;
}

