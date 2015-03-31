/* File:   config.h
 * Author: Jordan Argyle, Trevor Buckner, Jordan Calvert, Mikkel Unrau
 * ME 495R: Mechatronics

 * This header file contains all our pin and periferal configurations. */

#ifndef CONFIG_H
#define	CONFIG_H
#endif
// Required because sometimes wouldn't recognize the macros used, but at compile time it should.
#ifndef _T1IE
#include <p24FJ128GA202.h>
#endif

/*-------------------------------------------------------------------------------------------------
 1a) Pin configuration - Specifies which pins are used for what. A pin inventory in main.c:

 REFs: ANSA/B:         Registers, which are
       11-1/2 (p169)   in the data sheet
 * For function numbers: Table 11-4 (p174)
-------------------------------------------------------------------------------------------------*/
void PinConf()
{
    OSCCONbits.SOSCEN = 0;
    //OSCCONbits.

    TRISA = 0x0;    // output for all port A (0000), 4 A bits
    TRISB = 0x0000; // output for all port B (0000 0000 0000 0000), 16 B bits
    ANSA = 0x0;     // turn off analog input on all pins, port A
    ANSB = 0x0000;  // turn off analog input on all pins, port B

    TRISBbits.TRISB7 = 1;   // Set Pin 16 to input (CN23) for UltraSonicL
    TRISBbits.TRISB8 = 1;   // Set Pin 17 to input (CN22) for UltraSonicR
    TRISAbits.TRISA0 = 1;   // Set Pin  2 to input (RA0)  for LimitSwitchL
    TRISBbits.TRISB10 = 1;  // Set Pin 21 to input (RB10) for LimitSwitchR

    ANSBbits.ANSB13 = 1;
    ANSBbits.ANSB14 = 1;
    TRISBbits.TRISB13 = 1;  // Set Pin 25 to input (RB13) for LEDL
    TRISBbits.TRISB14 = 1;  // Set Pin 26 to input (RB14) for LEDR
    // Set Pin 14 to analog input (RB5) for PhotoDiode-FL
    //ANSBbits.ANSB5 = 1;
    TRISBbits.TRISB5 = 0;
    // Set Pin 15 to analog input (RB6) for PhotoDiode-BR
    //ANSBbits.ANSB6 = 1;
    TRISBbits.TRISB6 = 0;

    // Pin 2: Left limit switch (RA0). Digital input.
    // Pin 3: Trigger for Ultrosonics (RA1). Digital output.
    // Pin 4: Serial (UART) communciation for debugging (RB0). Digital output.
    #ifdef TESTING
      _RP0R = 3; //Assign RP0 to function 3 (UTX1)
    #endif
    // Pin 5: Ball release solenoid (RB1). Digital output.
    // Pin 6: Shooter motors. PWM 3.
    _RP2R = 15; //Assign RP2 to function 15 (OC3)
    // Pin 7: Servo motor for collection rod. PWM 2.
    _RP3R = 14; //Assign RP3 to function 14 (OC2)
    // Pin 14: Photodiode Input Front Left (RB5). A/D analog input
    // Pin 15: Photodiode Input Back Right (RB6). A/D analog input.
    // Pin 16: Echo (return) for Left Ultrasonic (RB7). Digital input.
    // Pin 17: Echo (return) for Right Ultrasonic (RB8). Digital input.
    // Pin 18: Output to step size on Pololu drivers (RB9). Digital out.
    // Pin 21: Right limit switch (RB10). Digital input.
    // Pin 23: To Pololu contollers for stepper motors. PWM 1.
    _RP12R = 13; //Assign RP12 to function 13 (OC1)
    // Pin 24: Direction of right wheel (RB13). Digital output.
    // Pin 25: Direction of left wheel  (RB14). Digital output.
    // Pin 26: "Sleep" mode on motor driver chips (RB15). Digital output.

    // Initialize pins
    LATBbits.LATB5 = 1; // Initialize WheelR Dir to 0
    LATBbits.LATB6 = 1; // Initialize WheelL Dir to 0
    LATBbits.LATB15 = 1; // Initialize Sleep to 1 (turns off sleep mode)
    LATBbits.LATB9 = 1;  // Initialize step size (M0) to half step (Stalls if left unconnected)
    LATBbits.LATB1 = 0;  // Initialize ball release solenoid to off (out)
    LATBbits.LATB5 = 1;
    LATBbits.LATB6 = 1;

    // Configure CN interrupt
    _CN23IE = 1;  // Enable CN on pin 16 (CNEN1 register)
    _CN23PUE = 0; // Disable pull-up resistor (CNPU1 register)
    _CNIP = 6;    // Set CN interrupt priority (IPC4 register)
    _CNIF = 0;    // Clear interrupt flag (IFS1 register)
    _CNIE = 1;    // Enable CN interrupts (IEC1 register)

    // Configure CN interrupt
    _CN22IE = 1;  // Enable CN on pin 17 (CNEN1 register)
    _CN22PUE = 0; // Disable pull-up resistor (CNPU1 register)
    _CNIP = 6;    // Set CN interrupt priority (IPC4 register)
    _CNIF = 0;    // Clear interrupt flag (IFS1 register)
    _CNIE = 1;    // Enable CN interrupts (IEC1 register)
}

/*-------------------------------------------------------------------------------------------------
 1b) PWM and Timer Configuration - Sets up our PWM output
 Called from main() intro section.

 REFs:   OSxCON1  ,   OSxCON2  ,   OCxR,  OCxRS ,    TxCON        Registers, which are
       15-1 (p216), 15-2 (p218), 15.3.2,3 (p213), 13-1 (p202)     in the data sheet
-------------------------------------------------------------------------------------------------*/
void PWMConf()
{
    /* ---- Configure OC1 (Pin 23--RP12) for stepper motors ---- */

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
    T2CONbits.TCS = 0;   // Internal clock
    T2CONbits.TCKPS = 0; // Don't prescale100
    TMR2 = 0;            // Timer set to 0
    T2CONbits.TON = 1;   // Timer is on
    PR2 =  3999;         // Timer period

    /* ---- Configure OC2 (Pin 7--RP3)to move servo to break IR beam for ball collection ---- */

    OC2CON1 = 0;    // Clear OC3 configuration bits
    OC2CON2 = 0;    // Clear OC3 configuration bits
    OC2CON1bits.OCTSEL = 0b010;     // Set it to use timer 4. Sets base for period.
    OC2CON2bits.SYNCSEL = 0b01110;  // Set sync select to timer 3. Sets comparisons and the "beat".
    OC2CON1bits.OCM = 0b110;        // Edge-Aligned PWM mode
    OC2R = 4000;    // Initialize to duty cycle of 20%. Set when used.

    // Set up Timer 4
    T4CONbits.TCS = 0;   // Internal clock
    T4CONbits.TCKPS = 0; // Don't prescale100
    TMR4 = 0;            // Timer set to 0
    T4CONbits.TON = 1;   // Timer is on
    PR4 =  19999;        // Timer period

    /* ---- Configure OC3 (Pin 22--RP11) to turn the shooter motors on with a MOSFET ---- */

    OC3CON1 = 0;    // Clear OC3 configuration bits
    OC3CON2 = 0;    // Clear OC3 configuration bits
    OC3CON1bits.OCTSEL = 0b001;     // Set it to use timer 3. Sets base for period.
    OC3CON2bits.SYNCSEL = 0b01101;  // Set sync select to timer 3. Sets comparisons and the "beat".
    OC3CON1bits.OCM = 0b110;        // Edge-Aligned PWM mode
    OC3R = 2000;    // Initialize with duty cycle of 50%

    // Set up Timer 3
    T3CONbits.TCS = 0;   // Internal clock
    T3CONbits.TCKPS = 0; // Don't prescale100
    TMR3 = 0;            // Timer set to 0
    T3CONbits.TON = 1;   // Timer is on
    PR3 = 4999;          // Timer period

    /* ---- Configure other timers that we use ---- */

    //Set up timer for determining how long to perform operations
    T1CONbits.TON = 1;      //Don't turn Timer 1 on until we need it
    T1CONbits.TCKPS = 0b11; //Prescale by 256
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

    _CH0NA = 0b000;     // Choose GND (pin 20) as negative input
    _CH0SA = 0b00110; // Choose AN6 (pin 25) as positive input. Less reliable
                        // than using AD1CSSL/H, so we don't use it

    // AD1CON1 register (22-1)
    _ADON = 1;      // AD1CON1<15> -- A/D on?
                    // 1=On
    _ADSIDL = 1;    // AD1CON1<13> -- A/D stops while in idle mode?
                    // 0=yes
    _MODE12 = 1;    // AD1CON1<10> -- 12-bit or 10-bit?
                    // 1=12
    _FORM = 0b00;   // AD1CON1<9:8> -- Output format
                    // 00=Abs decimal, unsigned
                    // 10=Abs fractional, unsigned
    _SSRC = 0b0111; // AD1CON1<7:4> -- Sample clock source select
                    // 0111=Auto conversion, internal counter
    _ASAM = 1;    // AD1CON1<2> -- When to sample
                    // 1=Continuous auto sampling

    // AD1CSSL/H registers (22-9 and 22-8)
    //AD1CSSL = 0;    // AD1CSSL<15:0> -- Select lower channels to scan
                    // 0=all off, since we'll turn on the ones we want individually
    //AD1CSSH = 0;    // AD1CSSH<15:0> -- Select upper channels to scan
                    // 0=all off; we'll turn on the ones we want individually
    //_CSS12 = 1;     // Turn on AN12 (Pin 15) to sample

    // AD1CON2 register (22-2)
    _PVCFG = 0;         // AD1CON2<15:14> -- Set positive voltage reference
                        // 0=Use VDD as positive ref voltage
    _NVCFG0 = 0;         // AD1CON2<13> -- Set negative voltage reference
                        // 0=Use VSS as negative ref voltage
    _BUFREGEN = 1;      // AD1CON2<11> -- A/D buffer register enable?
                        // 1=enabled. Results stored using channel indexed
                        // mode -- AN1 result is stored in ADC1BUF1, AN2 result
                        // is stored in ADC1BUF2, etc.
    _CSCNA = 0;         // AD1CON2<10> -- Scan inputs?
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