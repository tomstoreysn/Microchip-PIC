/*******************************************************************************
 * File:    types_consts.h
 * Author:  Tom Storey <github.com/tomstorey>
 * 
 * Description
 *      Types and other constants used in source files in this repository.
 * 
 * Revision History
 * Date         Author      Detail
 * 13/08/2017   Tom         Created
 ******************************************************************************/

#ifndef XC_HEADER_TYPES_H
#define	XC_HEADER_TYPES_H

#include <xc.h>

/* 
 * Named constants
 */
#define FALSE 0
#define TRUE 1
#define LOW 0
#define HIGH 1
#define OUTPUT 0
#define INPUT 1
#define OFF 0
#define ON 1

#define PERIPH_ON 0
#define PERIPH_OFF 1

/* 
 * More descriptive types
 */
typedef unsigned char char_t;
typedef unsigned char uint8_t;
typedef unsigned int uint16_t;
typedef unsigned long uint32_t;
typedef signed char int8_t;
typedef signed int int16_t;
typedef signed long int32_t;

#endif	/* XC_HEADER_TYPES_H */

