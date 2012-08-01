/*
 * main.h
 *
 *  Created on: Jul 19, 2012
 *      Author: rasmus
 */

#ifndef MAIN_H_
#define MAIN_H_

/**
 * Function to send a number of bits to the HT1632
 * @param data
 * @param bits
 */
void HTsend(uint16_t data, uint8_t bits);

/**
 * Function to fill the entire LED array with the data in global var leds[]
 */
void HTsendscreen();

/**
 * Function to send a command to the HT1632
 */
void HTcommand(uint16_t data);

/**
 * Function to set the brightness of the LED array
 */
void HTbrightness(uint8_t b);

void SingleLedOn(uint8_t x, uint8_t y);
void SingleLedOff(uint8_t x, uint8_t y);
void UpdateLife();
void RandomLeds();
void ResetGameOfLife();


#endif /* MAIN_H_ */
