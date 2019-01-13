 /************************************************************************/
 /* main.c - Milestone 2 Code                                            */
 /*                                                                      */
 /* Kevin Duong                                                          */
 /* n9934731	                                                         */
 /*                                                                      */
 /*                                                                      */
 /************************************************************************/ 

 /************************************************************************/
 /* INCLUDED LIBRARIES/HEADER FILES                                      */
 /************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>

#include <stdio.h>

#include "serial.h"

#define TOP 255


/************************************************************************/
/* Initialisation                                                       */
/************************************************************************/
void init() 
{
	cli();

	// Configure system clock for 16 MHz
	CLKPR = 0x80;	// Prescaler change enable
	CLKPR = 0x00;	// Prescaler /1, 16 MHz

	DDRF &= 0b00001111;		// Set PORTF 7-4 as inputs (PBs)
	DDRD |= 0b11110000;
	DDRB |= 0b01000000;		// Set PORTB 6 (JOUT) as output (LEDs)
	PORTD &= 0b00001111;	// turn LEDs off

	// Initialise Timer4 for 8-bit PWM
	// Using OC4D, connected to LED4
	//TCCR4B = 0x0B;  //prescaler 1024, ~61Hz
	TCCR4B = 0x03;  //prescaler 1024, ~15.625Hz
	TCCR4A = 0b00100001;   // Clear on CMP, enable PWM
	OCR4C = TOP;    // set top to 0xFF (255)
	OCR4B = 0x80;   // initialise to 50% duty cycle
	TCNT4 = 0x00;  // reset counter

	serial_init();	// Initialise USB serial interface (debug)

	sei();
}/*init*/

/************************************************************************/
/* MAIN LOOP (CODE ENTRY)                                               */
/************************************************************************/
int main(void) 
{
	init();

	// main program loop
    while(1)
	{
		  if (~PINF & 0b00010000)   // S1
		  {
		    OCR4B = 0x57;	        //PWM -> 34% duty cycle 87 decimal			
		  	PORTD |= 0b00010000;    // turn LED1
		  }

		  if (~PINF & 0b00100000)   // S2
		  {
		      OCR4B = 0xBB;	        //PWM -> 73% duty cycle 187 decimal
		      PORTD |= 0b00100000;    // turn LED2 on
		  }

		  if (~PINF & 0b01000000)   // S3
		  {
		      OCR4B = 0xFD;	        //PWM -> 99% duty cycle 253 decimal
		      PORTD |= 0b01000000;    // turn LED3 on
		  }

		  if (PINF & 0b01110000)    // none of push button S1-S3 pressed
		     PORTD &= 0b10001111;    // turn LEDs 1-3 off
	} /*while*/

}/*end main*/
