#ifndef _PLATFORM_H_
#define _PLATFORM_H_

/******************************Global includes*********************************/
#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
/******************************************************************************/

/********************Defining the type of the MCU******************************/
#define MCU_TYPE DOIT_ESP_32_DIVKIT_V1
/******************************************************************************/

/********************Data types based on the type of the MCU*******************/
#if (DOIT_ESP_32_DIVKIT_V1 == MCU_TYPE)

typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned long uint32;
typedef signed char sint8;
typedef signed short sint16;
typedef signed long sint32;
typedef float float32;

#endif
/******************************************************************************/

#endif