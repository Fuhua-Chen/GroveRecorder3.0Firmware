/**************************************************************************//**
 * @file     ConfigIO.h
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 14/09/25 11:00a $
 * @brief    Header file of IO configuration.
 *           This file could be generated by NuKeilIDE tool. 
 *
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#ifndef _CONFIGIO_H_
#define _CONFIGIO_H_

#include "Platform.h"

//%{CodeGen_Block_Start}
#define KEY_DEBOUNCE_TIME		(0.0500)	// unit:s
#define KEY_PRESS_TIME			(1.0000)	// unit:s

//------------------------------------------------------------------------------------------------//
// Configurations of Matrix Keys
//------------------------------------------------------------------------------------------------//
// Keypad Matrix Mapping
//
//	(0 x 0)

#define MATRIX_INPUT_PINS_COUNT			(0)	// in:0, out:0
#define MATRIX_OUTPUT_PINS_COUNT		(0)
#define MATRIX_KEY_COUNT				(MATRIX_INPUT_PINS_COUNT*MATRIX_OUTPUT_PINS_COUNT)

#define MATRIX_PORTA_INPUT_PINS_MASK	(0)
#define MATRIX_PORTA_OUTPUT_PINS_MASK	(0)	// BIT1~BITn

#define MATRIX_PORTB_INPUT_PINS_MASK	(0)
#define MATRIX_PORTB_OUTPUT_PINS_MASK	(0)	// BIT1~BITn

//------------------------------------------------------------------------------------------------//
// Configurations of Input Keys
//------------------------------------------------------------------------------------------------//
#define TRIGGER_KEY_COUNT	(6)

#define TG1_PIN_MASK		BIT0
#define TG2_PIN_MASK		BIT1
#define TG3_PIN_MASK		BIT2
#define TG4_PIN_MASK		BIT3
#define TG5_PIN_MASK		BIT6
#define TG6_PIN_MASK		BIT7

//TRIGGER_PORT
#define TRIGGER_PORTA_PINS_MASK		(0)
#define TRIGGER_PORTB_PINS_MASK		(TG1_PIN_MASK|TG2_PIN_MASK|TG3_PIN_MASK|TG4_PIN_MASK| \
		TG5_PIN_MASK|TG6_PIN_MASK)

//------------------------------------------------------------------------------------------------//
// Configurations of Output Keys
//------------------------------------------------------------------------------------------------//
#define OUT_KEY_COUNT		(6)

//OUT1
#define OUT1_PORT				PA
#define OUT1_PIN_MASK			BIT12
//OUT2
#define OUT2_PORT				PA
#define OUT2_PIN_MASK			BIT13
//OUT3
#define OUT3_PORT				PA
#define OUT3_PIN_MASK			BIT14
//OUT4
#define OUT4_PORT				PA
#define OUT4_PIN_MASK			BIT15
//OUT5
#define OUT5_PORT				PB
#define OUT5_PIN_MASK			BIT4
//OUT6
#define OUT6_PORT				PB
#define OUT6_PIN_MASK			BIT5
//OUTPUT_PORT
#define OUTPUT_PORTA_PINS_MASK		(OUT1_PIN_MASK|OUT2_PIN_MASK|OUT3_PIN_MASK|OUT4_PIN_MASK)
#define OUTPUT_PORTB_PINS_MASK		(OUT5_PIN_MASK|OUT6_PIN_MASK)

extern void OutputPin_Set(GPIO_T *pPort, UINT16 u16PinMask, UINT8 u8Value);

#define	OUT1(b)		OutputPin_Set(OUT1_PORT,OUT1_PIN_MASK,b)
#define	OUT2(b)		OutputPin_Set(OUT2_PORT,OUT2_PIN_MASK,b)
#define	OUT3(b)		OutputPin_Set(OUT3_PORT,OUT3_PIN_MASK,b)
#define	OUT4(b)		OutputPin_Set(OUT4_PORT,OUT4_PIN_MASK,b)
#define	OUT5(b)		OutputPin_Set(OUT5_PORT,OUT5_PIN_MASK,b)
#define	OUT6(b)		OutputPin_Set(OUT6_PORT,OUT6_PIN_MASK,b)

//------------------------------------------------------------------------------------------------//
// Configurations of KEY (key matrix) Handlers
//------------------------------------------------------------------------------------------------//
#define KEYR		0x10000000
#define KEYF		0x20000000
#define KEYP		0x40000000


//------------------------------------------------------------------------------------------------//
// Configurations of TG Handlers
//------------------------------------------------------------------------------------------------//
#define TGR			0x10000000
#define TGF			0x20000000
#define TGP			0x40000000

//TG1
#define TG1 		(1)
#define TG1R		(TG1|TGR)
#define TG1F		(TG1|TGF)
#define TG1P		(TG1|TGP)
//TG2
#define TG2 		(2)
#define TG2R		(TG2|TGR)
#define TG2F		(TG2|TGF)
#define TG2P		(TG2|TGP)
//TG3
#define TG3 		(3)
#define TG3R		(TG3|TGR)
#define TG3F		(TG3|TGF)
#define TG3P		(TG3|TGP)
//TG4
#define TG4 		(4)
#define TG4R		(TG4|TGR)
#define TG4F		(TG4|TGF)
#define TG4P		(TG4|TGP)
//TG5
#define TG5 		(5)
#define TG5R		(TG5|TGR)
#define TG5F		(TG5|TGF)
#define TG5P		(TG5|TGP)
//TG6
#define TG6 		(6)
#define TG6R		(TG6|TGR)
#define TG6F		(TG6|TGF)
#define TG6P		(TG6|TGP)

//------------------------------------------------------------------------------------------------//
// Configurations of In-Out keys
//------------------------------------------------------------------------------------------------//
extern void Default_KeyHandler(UINT32 u32Param);


#define DECLARE_MATRIX_KEY() \
S_KEYPAD_KEY_HANDLER const g_asMatrixKeyHandler[] =  \
{  \
	{0, 0, 0, 0, 0, 0, 0}  \
	}; \
	const UINT8 g_au8MatrixKeyStateIndex[] = {0};

#if (MATRIX_OUTPUT_PINS_COUNT>0)	// not create if MATRIX_KEY_COUNT==0
#define DECLARE_MATRIX_KEY_BUF()	UINT16 g_u16aKeyPinValueBuf[MATRIX_OUTPUT_PINS_COUNT];
#else
#define DECLARE_MATRIX_KEY_BUF()
#endif

//------------------------------------------------------------------------------------------------//

#define	DECLARE_TRIGGER_KEY() \
S_KEYPAD_TGR_HANDLER const g_asTriggerKeyHandler[] =  \
{ \
	{Default_KeyHandler, TG1F, BIT0, KEYPAD_GPIOB, KEYPAD_FALLING}, \
	{Default_KeyHandler, TG1R, BIT0, KEYPAD_GPIOB, KEYPAD_RISING}, \
	{Default_KeyHandler, TG1P, BIT0, KEYPAD_GPIOB, KEYPAD_PRESSING}, \
	{Default_KeyHandler, TG2F, BIT1, KEYPAD_GPIOB, KEYPAD_FALLING}, \
	{Default_KeyHandler, TG2R, BIT1, KEYPAD_GPIOB, KEYPAD_RISING}, \
	{Default_KeyHandler, TG2P, BIT1, KEYPAD_GPIOB, KEYPAD_PRESSING}, \
	{Default_KeyHandler, TG3F, BIT2, KEYPAD_GPIOB, KEYPAD_FALLING}, \
	{Default_KeyHandler, TG3R, BIT2, KEYPAD_GPIOB, KEYPAD_RISING}, \
	{Default_KeyHandler, TG3P, BIT2, KEYPAD_GPIOB, KEYPAD_PRESSING}, \
	{Default_KeyHandler, TG4F, BIT3, KEYPAD_GPIOB, KEYPAD_FALLING}, \
	{Default_KeyHandler, TG4R, BIT3, KEYPAD_GPIOB, KEYPAD_RISING}, \
	{Default_KeyHandler, TG4P, BIT3, KEYPAD_GPIOB, KEYPAD_PRESSING}, \
	{Default_KeyHandler, TG5F, BIT6, KEYPAD_GPIOB, KEYPAD_FALLING}, \
	{Default_KeyHandler, TG5R, BIT6, KEYPAD_GPIOB, KEYPAD_RISING}, \
	{Default_KeyHandler, TG5P, BIT6, KEYPAD_GPIOB, KEYPAD_PRESSING}, \
	{Default_KeyHandler, TG6F, BIT7, KEYPAD_GPIOB, KEYPAD_FALLING}, \
	{Default_KeyHandler, TG6R, BIT7, KEYPAD_GPIOB, KEYPAD_RISING}, \
	{Default_KeyHandler, TG6P, BIT7, KEYPAD_GPIOB, KEYPAD_PRESSING}, \
	{0, 0, 0, 0, 0}  \
	}; \
	const UINT8 g_au8TriggerKeyStateIndex[] = {3,6};


//%{CodeGen_Block_End}

extern void Default_KeyHandler(UINT32 u32Param);

//------------------------------------------------------------------------------------------------//
// Configurations of Touch Keys
//------------------------------------------------------------------------------------------------//
#define TOUCHR		0x10000000
#define TOUCHF		0x20000000
#define TOUCHP		0x40000000

// TOUCH1
#define TOUCH1 		(1)
#define TOUCH1R		(TOUCH1|TOUCHR)
#define TOUCH1F		(TOUCH1|TOUCHF)
#define TOUCH1P		(TOUCH1|TOUCHP)
// TOUCH2
#define TOUCH2 		(2)
#define TOUCH2R		(TOUCH2|TOUCHR)
#define TOUCH2F		(TOUCH2|TOUCHF)
#define TOUCH2P		(TOUCH2|TOUCHP)
// TOUCH3
#define TOUCH3 		(3)
#define TOUCH3R		(TOUCH3|TOUCHR)
#define TOUCH3F		(TOUCH3|TOUCHF)
#define TOUCH3P		(TOUCH3|TOUCHP)
// TOUCH4
#define TOUCH4 		(4)
#define TOUCH4R		(TOUCH4|TOUCHR)
#define TOUCH4F		(TOUCH4|TOUCHF)
#define TOUCH4P		(TOUCH4|TOUCHP)
// TOUCH5
#define TOUCH5 		(5)
#define TOUCH5R		(TOUCH5|TOUCHR)
#define TOUCH5F		(TOUCH5|TOUCHF)
#define TOUCH5P		(TOUCH5|TOUCHP)
// TOUCH6
#define TOUCH6 		(6)
#define TOUCH6R		(TOUCH6|TOUCHR)
#define TOUCH6F		(TOUCH6|TOUCHF)
#define TOUCH6P		(TOUCH6|TOUCHP)
	
#define TOUCH_KEY_COUNT			(0)

#define TOUCH_CLR_CAP_MASK		(SYS_GPB_MFP_PB0MFP_Msk|SYS_GPB_MFP_PB1MFP_Msk|SYS_GPB_MFP_PB2MFP_Msk|SYS_GPB_MFP_PB3MFP_Msk|SYS_GPB_MFP_PB6MFP_Msk|SYS_GPB_MFP_PB7MFP_Msk)
#define TOUCH_CFG_CAP_MASK		(SYS_GPB_MFP_PB0MFP_CMP0|SYS_GPB_MFP_PB1MFP_CMP1|SYS_GPB_MFP_PB2MFP_CMP2|SYS_GPB_MFP_PB3MFP_CMP3|SYS_GPB_MFP_PB6MFP_CMP6|SYS_GPB_MFP_PB7MFP_CMP7)
	
#define TOUCH_KEY_PINS_MASK		(BIT0|BIT1|BIT2|BIT3|BIT6|BIT7)
	
#define TOUCH_KEY_HANDLE_COUNT	(18)
	
#if( TOUCH_KEY_COUNT > 0 )
#define DECLARE_TOUCH_KEY() 								\
S_KEYPAD_TOUCH_HANDLER const g_asTouchKeyHandler[] =  		\
{  															\
	{Default_KeyHandler, TOUCH1F, BIT0, KEYPAD_FALLING},	\
	{Default_KeyHandler, TOUCH1R, BIT0, KEYPAD_RISING},		\
	{Default_KeyHandler, TOUCH1P, BIT0, KEYPAD_PRESSING},	\
	{Default_KeyHandler, TOUCH2F, BIT1, KEYPAD_FALLING},	\
	{Default_KeyHandler, TOUCH2R, BIT1, KEYPAD_RISING},		\
	{Default_KeyHandler, TOUCH2P, BIT1, KEYPAD_PRESSING},	\
	{Default_KeyHandler, TOUCH3F, BIT2, KEYPAD_FALLING},	\
	{Default_KeyHandler, TOUCH3R, BIT2, KEYPAD_RISING},		\
	{Default_KeyHandler, TOUCH3P, BIT2, KEYPAD_PRESSING},	\
	{Default_KeyHandler, TOUCH4F, BIT3, KEYPAD_FALLING},	\
	{Default_KeyHandler, TOUCH4R, BIT3, KEYPAD_RISING},		\
	{Default_KeyHandler, TOUCH4P, BIT3, KEYPAD_PRESSING},	\
	{Default_KeyHandler, TOUCH5F, BIT6, KEYPAD_FALLING},	\
	{Default_KeyHandler, TOUCH5R, BIT6, KEYPAD_RISING},		\
	{Default_KeyHandler, TOUCH5P, BIT6, KEYPAD_PRESSING},	\
	{Default_KeyHandler, TOUCH6F, BIT7, KEYPAD_FALLING},	\
	{Default_KeyHandler, TOUCH6R, BIT7, KEYPAD_RISING},		\
	{Default_KeyHandler, TOUCH6P, BIT7, KEYPAD_PRESSING},	\
	{0,0,0,0}												\
	}; 														\
	const UINT8 g_au8TouchKeyStateIndex[] = {0};
#else
#define DECLARE_TOUCH_KEY() 		
#endif
	
#if ( TOUCH_KEY_HANDLE_COUNT > 0 && TOUCH_KEY_COUNT > 0 )   
#define DECLARE_TOUCH_KEY_BUF()	UINT16 g_u16aTouchPinValueBuf[TOUCH_KEY_HANDLE_COUNT]; 
#else
#define DECLARE_TOUCH_KEY_BUF()
#endif

#if ( TOUCH_KEY_COUNT > 0 )
#define DECLARE_TOUCH_THRESHOLD()							\
S_KEYPAD_TOUCH_THRESHOLD const KEYPAD_TOUCH_THRESHOLD[]= 	\
{															\
	{ 800, 350}, { 1200, 200}, { 750, 300}, { 800, 350},	\
	{ 800, 350}, { 750, 300}, { 800, 350}, { 1200, 200}		\
};
#else
#define DECLARE_TOUCH_THRESHOLD()
#endif
	
#endif