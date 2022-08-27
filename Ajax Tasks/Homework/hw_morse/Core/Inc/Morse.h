/**
  ******************************************************************************
  * @file           : Morse.h
  * @brief          : File which contains morse functions
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


#ifndef __MORSE_H__
#define __MORSE_H__


#define SIGNAL_SHORT 50
#define SIGNAL_LONG 150 // 50*3
#define SIGNAL_SYMBOL_PAUSE 70
#define SIGNAL_LETTER_PAUSE 100
#define SIGNAL_WORD_PAUSE SIGNAL_SHORT*7

#define ALPHABET_NUM 27
#define WORD_LENGHT 4
char Detect_Letter(char* word, int letternum);
void Morse_Simulator(char* word);

void Short_Signal(int loop);

void Long_Signal(int loop);

#endif
