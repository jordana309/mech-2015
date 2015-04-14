/* File:   shooting.h
 * Authors: Jordan Argyle, Trevor Buckner, Jordan Calvert, Mikkel Unrau
 * Created on March 17, 2015, 10:51 AM

 * Shooter functions - Easy way to perform repeated shooter commands.
 * We have a rod that will break the IR beam, so we need functions to move the rod.
 * We also need a function to turn on our ball shooting wheels, and to pull back the solonoid that
 *    holds the balls. We use processingTask (an int) to control this. */

#ifndef SHOOTING_H
#define	SHOOTING_H
#endif


//void rodRetract()
//{
//    processingTask = 1;
//    OC2R = 2000;     // Set pulse width to 500 microSec (90 deg right)
//    delay(10000, 1); // Wait for 0.625 seconds, halting the rest of the program
//}
//
///*-------------------------------------------------------------------------------------------------
// Move rod left - this function flips the beam breaking rod to the left.
// * Called from:
//-------------------------------------------------------------------------------------------------*/
//void rodLeft()
//{
//    processingTask = 1;
//    OC2R = 8333;        //Set pulse width to 2083 microsec (60 deg left)
//    //OR 9600 TO GET FULL 90 DEG LEFT FOR BIGGER SWEEP
//    //OC2R = 7700;     // Set pulse width to 1950 microSec (45 deg left) //MAY HAVE CHANGED BY x4
//    delay(10000, 1); // Wait for 0.625 seconds, halting the rest of the program
//}
//
///*-------------------------------------------------------------------------------------------------
// Move rod right - this function flips the beam breaking rod to the right.
// * Called from:
//-------------------------------------------------------------------------------------------------*/
//void rodRight()
//{
//    processingTask = 1;
//    OC2R = 7065;     // Set pulse width to 1766 microSec (30 deg left)
//    //OR 7700 TO GET 45 DEG LEFT FOR BIGGER SWEEP
//   // OC2R = 3900;     // Set pulse width to 975 microSec (45 deg right)
//    delay(10000, 1); // Wait for 0.625 seconds, halting the rest of the program
//}

/*-------------------------------------------------------------------------------------------------
 Start Shooting - this function opens the MOSFET, allowing current to our high-speed motors to
    shoot the ping-pong ball. We need to wait to fire until it's at speed.
 * Called from:
-------------------------------------------------------------------------------------------------*/
void startShooting()
{
    LATBbits.LATB9 = 1; //Pin 18
    delay(16000, 1);     //Wait 1 seconds to get motor spinning up
}

/*-------------------------------------------------------------------------------------------------
 Stop Shooting - this function turns on the high-speed motors to shoot the ping-pong ball. We need to
    wait until it's at speed.
 * Called from:
-------------------------------------------------------------------------------------------------*/
void stopShooting()
{
    //OC3R = 0;        // Turn off MOSFET feeding shooter motors
    LATBbits.LATB9 = 0; //Pin 18
}

/*-------------------------------------------------------------------------------------------------
 elease ball - this function pulls back the solonoid to allow a ball through.
 * Called from:
-------------------------------------------------------------------------------------------------*/
void releaseBall()
{
    LATBbits.LATB15 = 1;     //Open solenoid to release ball
    delay(1280, 1);        //Wait for 0.08 seconds, just enough time to let one ball through
    LATBbits.LATB15 = 0;     //Close solenoid
}

void shootOnce()
{
    OC3R = 2000;     // Pulse MOSFET to allow current to motors to launch balls
    delay(16000, 1); // Wait for 1 second, halting the rest of the program
    OC3R = 0;      // Turn off shooter motors
}
