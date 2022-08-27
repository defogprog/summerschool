#include "Morse.h"
#include "main.h"

/**
  ******************************************************************************
  * @file           : Morse.c
  * @brief          : File which contains morse functionality
  ******************************************************************************
  * @attention
  *
  * Copyright LOLOLOL(c) 2022 prodamgaraj.
  * All rights reserved.
  *
  * If i find out that you use these lines of code without my permission, i will
  * track you down. Be aware.
  *
  *
  *
  ******************************************************************************
  */

/**
 * This is simple comparator which detects letters. The user defined char[] array (char *word) is compared
 * to the Alphabet_Lower[] and Alphabet_Upper[] arrays. When common chars (letter) detected between two arrays, function returns it.
 *
 * The process is segmented in parts:
 *
 * 1. Detect letter
 * 2. Play signals according to letter
 *
 * All user has to do is to write char *word and these functions will automatically convert them to morse signals.
 * THE Main function here (which assembles everything together and will be used by user) is Morse_Simulator(char *word).
 */
char Detect_Letter(char* word, int letternum)
{
	char Alphabet_Lower[] = { "abcdefghijklmnopqrstuvwxyz"};
	char Alphabet_Upper[] = { "ABCDEFGHIJKLMNOPQRSTUVWXYZ" };

    for (int i = 0; i < ALPHABET_NUM; i++)
    {
        if (word[letternum] == Alphabet_Lower[i])
        {
            return Alphabet_Lower[i];
        }

        if (word[letternum] == Alphabet_Upper[i])
        {
            return Alphabet_Upper[i];
        }

    }
}

/**
 * This function generates short signals. It's frequencies are
 * defined by SIGNAL_SHORT (for signal lenght) and SIGNAL_SYMBOL_PAUSE (for pause lenght between signals)
 *
 * The interesting part is that you can set how many signals in a row will be played. (int loop).
 *
 * 0 - only one signal
 * 1-n - n-times loop
 *
 * It's very easy to set up individual letters with this technique.
 */
void Short_Signal(int loop)
{
    if (loop == 0)
    {
    	HAL_GPIO_WritePin(LED_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
    	HAL_Delay(SIGNAL_SHORT);
    	HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
    	HAL_Delay(SIGNAL_SYMBOL_PAUSE);
    }
    else
    {
        for (int i = 0; i < loop; i++)
        {
        	HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
        	HAL_Delay(SIGNAL_SHORT);
        	HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
        	HAL_Delay(SIGNAL_SYMBOL_PAUSE);
        }
    }

}


/**
 * This function generates long signals. It's frequencies are
 * defined by SIGNAL_LONG (for signal lenght) and SIGNAL_SYMBOL_PAUSE (for pause lenght between signals)
 *
 * The interesting part is that you can set how many signals in a row will be played. (int loop).
 *
 * 0 - only 1 signal
 * 1-n - n-times loop
 *
 * It's very easy to set up individual letters with this technique.
 */
void Long_Signal(int loop)
{
    if (loop == 0)
    {
    	HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
    	HAL_Delay(SIGNAL_LONG);
    	HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
    	HAL_Delay(SIGNAL_SYMBOL_PAUSE);
    }
    else
    {
        for (int i = 0; i < loop; i++)
        {
        	HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
        	HAL_Delay(SIGNAL_LONG);
        	HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
        	HAL_Delay(SIGNAL_SYMBOL_PAUSE);
        }
    }

}

/**
 * This is preconfig for every english letter. As was said earlier, it's very easy to
 * set up this. re_result is assigned with return value of function Detect_Letter (explanation above).
 */
void Morse_Simulator(char* word)
{
    char re_result;

    for (int i = 0; i < WORD_LENGHT; i++)
    {
        re_result = Detect_Letter(word, i);

        switch (re_result)
        {
        case 'a':
        case 'A':
            // printf(" .-");
            Short_Signal(0);
            Long_Signal(0);

            break;
        case 'b':
        case 'B':
           // printf(" -...");
            Long_Signal(0);
            Short_Signal(3);

            break;
        case 'c':
        case 'C':
           // printf(" -.-.");
            Long_Signal(0);
            Short_Signal(0);
            Long_Signal(0);
            Short_Signal(0);

            break;
        case 'd':
        case 'D':
           // printf(" -..");
            Long_Signal(0);
            Short_Signal(2);

            break;
        case 'e':
        case 'E':
           // printf(" .");
            Short_Signal(0);

            break;
        case 'f':
        case 'F':
           // printf(" ..-.");
            Short_Signal(2);
            Long_Signal(0);
            Short_Signal(0);

            break;
        case 'g':
        case 'G':
           // printf(" --.");
            Long_Signal(2);
            Short_Signal(0);

            break;
        case 'h':
        case 'H':
           // printf(" ....");
            Short_Signal(4);

            break;
        case 'i':
        case 'I':
          //  printf(" ..");
            Short_Signal(2);

            break;
        case 'j':
        case 'J':
           // printf(" .---");
            Short_Signal(0);
            Long_Signal(3);

            break;
        case 'k':
        case 'K':
           // printf(" -.-");
            Long_Signal(0);
            Short_Signal(0);
            Long_Signal(0);

            break;
        case 'l':
        case 'L':
            //printf(" .-..");
            Short_Signal(0);
            Long_Signal(0);
            Short_Signal(2);

            break;
        case 'm':
        case 'M':
           // printf(" --");
            Long_Signal(2);

            break;
        case 'n':
        case 'N':
           // printf(" -.");
            Long_Signal(0);
            Short_Signal(0);

            break;
        case 'o':
        case 'O':
            //printf(" ---");
            Long_Signal(3);

            break;
        case 'p':
        case 'P':
           // printf(" .--.");
            Short_Signal(0);
            Long_Signal(2);
            Short_Signal(0);

            break;
        case 'q':
        case 'Q':
            // printf(" --.-");
            Long_Signal(2);
            Short_Signal(0);
            Long_Signal(0);

            break;
        case 'r':
        case 'R':
            // printf(" .-.");
            Short_Signal(0);
            Long_Signal(0);
            Short_Signal(0);

            break;
        case 's':
        case 'S':
            // printf(" ...");
            Short_Signal(3);

            break;
        case 't':
        case 'T':
            // printf(" -");
            Long_Signal(0);

            break;
        case 'u':
        case 'U':
            // printf(" ..-");
            Short_Signal(2);
            Long_Signal(0);

            break;
        case 'v':
        case 'V':
            // printf(" ...-");
            Short_Signal(3);
            Long_Signal(0);

            break;
        case 'w':
        case 'W':
            // printf(" .--");
            Short_Signal(0);
            Long_Signal(2);

            break;
        case 'x':
        case 'X':
            // printf(" -..-");
            Long_Signal(0);
            Short_Signal(2);
            Long_Signal(0);

            break;
        case 'y':
        case 'Y':
            // printf(" -.--");
            Long_Signal(0);
            Short_Signal(0);
            Long_Signal(2);

            break;
        case 'z':
        case 'Z':
            // printf(" --..");
            Long_Signal(2);
            Short_Signal(2);

            break;
        default:
            break;
        }
        HAL_Delay(SIGNAL_LETTER_PAUSE);
    }
}
