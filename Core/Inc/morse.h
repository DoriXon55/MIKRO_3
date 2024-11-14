/*
 * morse.h
 *
 *  Created on: Nov 14, 2024
 *      Author: doria
 */

#ifndef INC_MORSE_H_
#define INC_MORSE_H_

#define DOT_TIME      200    // Czas trwania kropki (200ms)
#define DASH_TIME     600    // Czas trwania kreski (600ms)
#define SYMBOL_SPACE  200    // Przerwa między symbolami (200ms)
#define LETTER_SPACE  600    // Przerwa między literami (600ms)
void delay_ms(uint32_t ms);
void sendMorseCode(const char* code);
void sendLetterMorse(char letter);
void sendMessageMorse(const char* message);

#endif /* INC_MORSE_H_ */
