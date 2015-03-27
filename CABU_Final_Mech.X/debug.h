/* File:   debug.h
 * Author: Emmerson, Jordan Argyle, Trevor Buckner, Jordan Calvert, Mikkel Unrau
 * ME 495R: Mechatronics

 * This is a header file for all the debugging functions and initilizations. We configure UART to
 * be able to communicat with our bluetooth module. The other functions in here help us more easily
 * convert data we're interested in into something that can be sent over bluetooth. */

// Code already here by MPLabX. Figured it'd be a good idea to keep it.
#ifndef DEBUG_H
#define	DEBUG_H
#endif

// Maybe need this. It often causes the compiler to break, and UART seems to work without it.
//#include <PIC24F_plib.h>

/* Configures the UART, so we can serially communicate with our bluetooth */
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

/* Allows us to send commands to the bluetooth and for those to be pulled in */
char UART1GetChar()
{
   while(IFS0bits.U1RXIF == 0); //wait for buffer to receive
   IFS0bits.U1RXIF = 0; //reset interrupt flag
   return U1RXREG;
}

/* This sends a character to the bluetooth. It immediately sends to the bluetooth module, which
 * buffers the data and sends when it gets an endline character. */
void UART1PutChar(char Ch)
{
    while(U1STAbits.UTXBF == 1); //wait for buffer to be empty
    U1TXREG = Ch;
}

// Useful string transmission functions

/* Converts a word into characters to send over serial */
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

/* Converts a float number into characters to send over serial */
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

/* Converts an int number into characters to send over serial */
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