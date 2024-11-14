/*
 * morse.c
 *
 *  Created on: Nov 14, 2024
 *      Author: doria
 */

#include "main.h"
#include "gpio.h"
#include "morse.h"
const char* morseCode[] = {
    ".-",    // A
    "-...",  // B
    "-.-.",  // C
    "-..",   // D
    ".",    // E
    "..-.",  // F
    "--.",   // G
    "....",  // H
    "..",    // I
    ".---",  // J
    "-.-",   // K
    ".-..",  // L
    "--",    // M
    "-.",    // N
    "---",   // O
    ".--.",  // P
    "--.-",  // Q
    ".-.",   // R
    "...",   // S
    "-",     // T
    "..-",   // U
    "...-",  // V
    ".--",   // W
    "-..-",  // X
    "-.--",  // Y
    "--..",  // Z
};

void delay_ms(uint32_t ms) {
    HAL_Delay(ms);  // Wykorzystuje funkcję HAL_Delay, która jest standardową funkcją w STM32 HAL
}
void sendLetterMorse(char letter) {
    if (letter >= 'A' && letter <= 'Z') {
        sendMorseCode(morseCode[letter - 'A']);  // Obsługuje wielkie litery
    } else if (letter >= 'a' && letter <= 'z') {
        sendMorseCode(morseCode[letter - 'a']);  // Obsługuje małe litery
    }
}

// Funkcja do wysyłania kodu Morse'a
void sendMorseCode(const char* code) {
    for (int i = 0; code[i] != '\0'; i++) {
        if (code[i] == '.') {
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);  // Włącz LED
            delay_ms(DOT_TIME);                                      // Czas trwania kropki
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET); // Wyłącz LED
        } else if (code[i] == '-') {
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);  // Włącz LED
            delay_ms(DASH_TIME);                                      // Czas trwania kreski
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET); // Wyłącz LED
        }

        delay_ms(SYMBOL_SPACE);  // Przerwa między symbolami
    }
}

// Funkcja do wysyłania całej wiadomości w kodzie Morse'a
void sendMessageMorse(const char* message) {
    while (*message) {
        if (*message != ' ') {  // Ignorujemy spacje
            sendLetterMorse(*message); // Wyślij kod Morsa dla litery
        }
        delay_ms(LETTER_SPACE);    // Przerwa między literami
        message++;  // Przejdź do następnej litery
    }
}
