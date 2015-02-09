/* 
 * File:   main.c
 * Author: Jordan Argyle, Trevor Buckner, Jordan Calvert, Mikkel Unrau
 * ME 495R: Mechatronics
 * Final Project
 * Created on February 08, 2015, 20.05
 * Last edited on 
 *
 * This program includes all the code for our final mechatronics project. We have sections for:
 *   1) Initial configuration
 *   2) Orientation in the first 5 seconds
 *   3) Initial navigation to ball dispenser
 *   4) Ball collection
 *   5) Navigation from ball collection to center of arena
 *   6) Target Acquisition
 *   7) Firing the balls
 *   8) Repeat
 */

/* Include the needed header */
#include<p24F16KA301.h>

// TODO: This is not implemented, but just a place holder.
/* Set up a compiling variable, so we can create two programs in here for both PICs */
 * DATACRUNCHER = Program for the PIC that acquires our data--sensor PIC
 * BOTRUNNER    = Program for the PIC that drives the motors
 * To use this, wrap the necessary code between #ifdef VAR and #endif */
#define DATACRUNCHER

/* Select our oscillator. This is the same for both programs */
_FOSCSEL(FNOSC_FRCDIV); // 8 MHz with postscaling

/*-------------------------------------------------------------------------------------------------
A/D Configuration Function

 * This function configures the A/D to read from two channels in auto conversion mode. Uses:
 * AD1CHS, AD1CON1, AD1CON2, AD1CON3, AD1CSSL, AD1CSSH   registers, which are
 *   22-5,    22-1,    22-2,    22-3,    22-9,    22-8   in data sheet.
-------------------------------------------------------------------------------------------------*/
void ADInit(void)
{
    //Set up A/D 1, reading on pin 15 (AN12).
    
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
    _CSS12 = 1;     // Turn on AN12 (Pin 15) to sample
    _CSS11 = 1;	    // Turn on AN11 (Pin 16) to sample

    // AD1CON2 register (22-2)
    _PVCFG = 0;         // AD1CON2<15:14> -- Set positive voltage reference
                        // 0=Use VDD as positive ref voltage
    _NVCFG = 0;         // AD1CON2<13> -- Set negative voltage reference
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

/*-------------------------------------------------------------------------------------------------
PWM Initialization Function

 * This function configures the PWM. Uses:
 * OSXCON1, OSXCON2, OCxR, TxCON    Registers, which are
 *    15-1,    15-2,                in data sheet
-------------------------------------------------------------------------------------------------*/
void PWMInit()
{
    // Configure OC2 (pin 4)
    OC2CON1 = 0;    // Clear OC2 configuration bits
    OC2CON2 = 0;    // Clear OC2 configuration bits
    OC2CON1bits.OCTSEL = 0b000;     // Set it to use timer 2. Sets base for period.
    OC2CON2bits.SYNCSEL = 0b01100;  // Set sync select to timer 2. Sets comparisons and the "beat".
    OC2CON1bits.OCM = 0b110;        // Edge-Aligned PWM mode

    // Do the same, but for OC3 (pin 5)
    OC3CON1 = 0;    // Clear OC3 configuration bits
    OC3CON2 = 0;    // Clear OC3 configuration bits
    OC3CON1bits.OCTSEL = 0b000;    // Set it to use timer 2. Sets base for period.
    OC3CON2bits.SYNCSEL = 0b01100; // Set sync select to timer 2. Sets comparisons and the "beat".
    OC3CON1bits.OCM = 0b110;       // Edge-Aligned PWM mode
    
    // PWM period = [Value+1]*Tcy*Prescaler value
    // On time (duty cycle) in ticks of the period timer. I'm using T2 as my
    // ticker, so this is out of PR2.
    OC2R = 2000;
    OC3R = 2000;
    
    // Set up the timer
    T2CONbits.TCS = 0; // Internal clock
    T2CONbits.TCKPS = 0; // Don't prescale100
    _T2IE = 1; // Enable the interrupt
    _T2IP = 4; // Set interrupt priority
    _T2IF = 0; // Clear interrupt flag
    TMR2 = 0; // Timer set to 0
    T2CONbits.TON = 1; // Timer is on
    PR2 =  3999; // Timer period
}

/*-----------------------------------------------------------------------------
 Timer Interrupt

 * This function is the interrupt handler for timer2.
-----------------------------------------------------------------------------*/
void _ISR _T2Interrupt(void)
{
    _T2IF = 0; // Reset the interrupt flag
    TMR2 = 0;  // Reset the timer
}

//------------------------------------------------------------------------
// Main Function
//------------------------------------------------------------------------
int main()
{
    // 1) Initial configuration: Configure the pins
    TRISA = 0x00; // output for all port A (0000 0000)
    TRISB = 0x000; // output for all port B (0000 0000 0000)
    ANSA = 0x0; // turn off analog input on all pins, port A
    ANSB = 0x0; // turn off analog input on all pins, port B
    // Pin 4: OC2, (RB2), to be used for PWM output 1
    // Pin 5: OC3, (RB1), to be used for PWM output 2
    // Pin 15: AN12 (RB12), to be used for Pot. input 1
    _TRISB12 = 1; // Set as input
    _ANSB12 = 1;  // Set as analogue
    // Pin 16: AN11 (RB13), to be used for Pot. input 2
    _TRISB13 = 1; // Set as input
    _ANSB13 = 1;  // Set as analogue
    _RCDIV = 0x0; // No postscaler for oscilator

    // Configure PWM, A/D
    PWMInit();
    ADInit();
    
    // 2) Orientation in the first 5 seconds
    // 3) Initial navigation to ball dispenser
    // 4) Ball collection
    // 5) Navigation from ball collection to center of arena
    // 6) Target Acquisition
    // 7) Firing the balls
    // 8) Repeat

    // Main program start
    while(1)
    {
        
    }

    return 0;
}

