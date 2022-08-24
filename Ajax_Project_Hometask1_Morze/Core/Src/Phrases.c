/*
 * Phrases.c
 *
 *  Created on: Aug 3, 2022
 *      Author: cheburiek
 */

#include "Phrases.h"

void greet(void)
{
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	HAL_Delay(800);

	//GREETING STARTS//
	//Word:Vitaiy//
	V();
	gap(1);
	I();
	gap(1);
	T();
	gap(1);
	A();
	gap(1);
	YU();
	gap(2);
	//GREETING ENDS//
}

void task(void)
{
	//TASK START//
	//Word:Zaraz//
	Z();
	gap(1);
	A();
	gap(1);
	R();
	gap(1);
	A();
	gap(1);
	Z();
	//Word:Ya//
	gap(2);
	YA();
	gap(2);
	//Word:Vykonuyu//
	V();
	gap(1);
	Y();
	gap(1);
	K();
	gap(1);
    O();
	gap(1);
	N();
	gap(1);
	U();
	gap(1);
	YU();
	gap(2);
	//Word:Robotu//
	R();
	gap(1);
	O();
	gap(1);
	B();
	gap(1);
	O();
	gap(1);
	T();
	gap(1);
	U();
	gap(2);
	//TASK ENDS//
}

void job(int times)
{
	for(int i = 1; i <= times; i++)
	{
	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	  HAL_Delay(500);
	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	  HAL_Delay(500);
	}
}

void done(void)
{
	//FINISHING START//
	//Word:Vykonano//
	V();
	gap(1);
	Y();
	gap(1);
	K();
	gap(1);
    O();
	gap(1);
	N();
	gap(1);
	A();
	gap(1);
	N();
	gap(1);
    O();
	gap(2);
	//FINISHING ENDS//
}

void bye(void)
{
	B();
	gap(1);
	U();
	gap(1);
	V();
	gap(1);
	A();
	gap(1);
	YOT();
	gap(1);
	T();
	gap(1);
	E();
	gap(2);
}
