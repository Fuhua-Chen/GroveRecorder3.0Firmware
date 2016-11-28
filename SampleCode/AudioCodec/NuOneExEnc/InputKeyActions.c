/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) Nuvoton Technology Corp. All rights reserved.                                              */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

// ---------------------------------------------------------------------------------------------------------
//	Functions:
//		- Direct trigger key and matrix key falling, rising, long pressing handling functions.
//			* Direct trigger key handling functions are referenced in a lookup table "g_asTriggerKeyHandler" (in "ConfigIO.h").
//			* Matrix key handling functions are reference in a lookup talbe "g_asMatrixKeyHandler" (in "ConfigIO.h").
//			* Both lookup tables are used by keypad library to call at key action being identified.
//		- Default trigger key and matrix key handler is "Default_KeyHandler()"
//
//	Reference "Readme.txt" for more information.
// ---------------------------------------------------------------------------------------------------------

#include "App.h"
#include "Keypad.h"
#include "stdio.h"

extern volatile UINT8 g_u8AppCtrl;
extern volatile BOOL playEndStatus;
extern volatile BOOL recEndStatus;

extern BOOL App_StartPlay(void);
extern BOOL App_StopPlay(void);
extern BOOL App_StopRec(void);
extern BOOL App_StartRec(void);
extern BOOL App_ProcessRec(void);
extern BOOL App_ProcessPlay(void);
extern void App_PowerDown(void);
extern void App_ReadVR(void);
extern void delay_ms(UINT32 ms);

void Record_KeypadHandler(UINT32 u32Param)
{
	if ( g_u8AppCtrl&(APPCTRL_PLAY|APPCTRL_PLAY_STOP) )
		return;

//Comment by lambor
//	if ( (g_u8AppCtrl&APPCTRL_RECORD) ==0 )
//		App_StartRec();
//	else
//		App_StopRec();

//Added by lambor
	if ( u32Param ==0 )
		App_StartRec();
	else
		App_StopRec();
}

void Playback_KeypadHandler(UINT32 u32Param)
{
	if ( g_u8AppCtrl&APPCTRL_RECORD )
	   return;
// Comment by lambor
//	if ( (g_u8AppCtrl&APPCTRL_PLAY) == 0 )
//		App_StartPlay();
//	else
//		App_StopPlay();

// Added by lambor
	if (u32Param == 0 ) {
		App_ReadVR();
		App_StartPlay();
	}
	else
		App_StopPlay();
}

void PowerDown_KeypadHandler(UINT32 u32Param)
{
	App_PowerDown();
}

void Default_KeyHandler(UINT32 u32Param)
{
	// Within this function, key state can be checked with these keywords:
	//	TGnF: direct trigger key n is in falling state(pressing)
	//	TGnR: direct trigger key n is in rising state(releasing)
	//	TGnP: direct trigger key n is in long pressing state
	//	KEYmF: matrix key m is in falling state(pressing)
	//	KEYmR: matrix key m is in rising state(releasing)
	//	KEYmP: matrix key m is in long pressing state
	// the maxium value of n is defined by "TRIGGER_KEY_COUNT" in "ConfigIO.h"
	// the maxium value of m is defined by "MATRIX_KEY_COUNT"  in "ConfigIO.h"
  static UINT8 IsRunning = 0;
	UINT32 u32Tmp = 0;
	
  if(IsRunning == 1)
	{
		return;
	}
	IsRunning = 1;
	
#if 1  //pull up resistance at Grove pins
		switch(u32Param)
		{
			case TG3R:	 //PB2
				printf("TG3R...\n\r");
			  //OUT5(1);
			  //Playback_KeypadHandler(1);  // Stop play
				break;
			case TG3F:
				printf("TG3F...\n\r");
				OUT5(0);
				printf("Start play\n\r");
				Playback_KeypadHandler(0);  // Start play
			
			
//				while((GPIO_GET_IN_DATA(PB) & BIT2 ) != BIT2)
//				{
//					if ( App_ProcessPlay() == FALSE )
//						App_StopPlay();
//				}	
//				printf("Stop play\n\r");
//				OUT5(1);
//				Playback_KeypadHandler(1);  // Stop play		

				while(1)
				{
					if(((GPIO_GET_IN_DATA(PB) & BIT2 ) == BIT2) && ( App_ProcessPlay() == FALSE ))
					{
							App_StopPlay();
							OUT5(1);
							Playback_KeypadHandler(1);  // Stop play
						  break;
					}
			  }
				break;
			case TG4R:	//PB3
				printf("TG4R...\n\r");
				//Record_KeypadHandler(1);   //Stop rec
				//OUT5(1);
				break;
			case TG4F:  // Start rec
				printf("TG4F...\n\r");
				OUT5(0);
				Playback_KeypadHandler(1);  // Stop play
				Record_KeypadHandler(0);   // Start rec
				printf("Start Rec.\n\r");
				while((GPIO_GET_IN_DATA(PB) & BIT3 ) != BIT3)
				{
					if ( App_ProcessRec() == FALSE )
						App_StopRec();
				}
				OUT5(1);
				printf("Stop Rec.\n\r");
				Record_KeypadHandler(1);   //Stop rec
				break;
			case TG5R:	 //PB6 multiplexing Button, record and play
				printf("TG5R...\n\r");
				OUT5(1);
				//Record_KeypadHandler(1);   //Stop rec
				break;
			case TG5F:
				printf("TG5F...\n\r");
				if(playEndStatus) {
					while( (GPIO_GET_IN_DATA(PB) & BIT6 ) != BIT6)
					{
						delay_ms(10);	 //10ms
						if( (u32Tmp++) >= 100)
						{
							//printf("Start Rec.\n\r");
							Record_KeypadHandler(0);   //Start rec
							while((GPIO_GET_IN_DATA(PB) & BIT6 ) != BIT6)
							{
								if ( App_ProcessRec() == FALSE )
									App_StopRec();
							}
							break;
						}
					}

					printf("Key5 press for %d*10ms.\n\r", u32Tmp);

					delay_ms(10);
					Record_KeypadHandler(1);  //Stop Rec
					delay_ms(10);

					if(u32Tmp < 100)
					{
						OUT5(0);
						Playback_KeypadHandler(0);
					}
				}
				break;
		}
#endif

#if 0     //No pull up resistance
		switch(u32Param)
		{
			case TG3F:	 //PB2
				printf("TG3F...\n\r");
				if(!playEndStatus) {
					OUT5(1);
					Playback_KeypadHandler(1);  // stop play
				}
				break;
			case TG3R:
				printf("TG3R...\n\r");
				if(playEndStatus) {
					Playback_KeypadHandler(0);  // start play
					OUT5(0);
				}
				break;
			case TG4F:	//PB3
				printf("TG4F...\n\r");
				if(!recEndStatus){
					Record_KeypadHandler(1);   //Stop rec
					OUT5(1);
				}
				break;
			case TG4R:
				printf("TG4R...\n\r");
				if(recEndStatus){
					OUT5(0);
					Record_KeypadHandler(0);   //Start rec
				}
				break;
			case TG5R:	 //PB6 multiplexing Button, record and play
				printf("TG5R...\n\r");
				OUT5(1);
				//Record_KeypadHandler(1);   //Stop rec
				break;
			case TG5F:
				printf("TG5F...\n\r");
				if(playEndStatus) {
					while( (GPIO_GET_IN_DATA(PB) & BIT6 ) != BIT6)
					{
						delay_ms(10);	 //10ms
						if( (u32Tmp++) >= 100)
						{
							//printf("Start Rec.\n\r");
							Record_KeypadHandler(0);   //Start rec
							while((GPIO_GET_IN_DATA(PB) & BIT6 ) != BIT6)
							{
								if ( App_ProcessRec() == FALSE )
									App_StopRec();
							}
							break;
						}
					}

					printf("Key5 press for %d*10ms.\n\r", u32Tmp);

					delay_ms(10);
					Record_KeypadHandler(1);  //Stop Rec
					delay_ms(10);

					if(u32Tmp < 100)
					{
						OUT5(0);
						Playback_KeypadHandler(0);
					}
				}
				break;
		}
#endif
  
	IsRunning = 0;
}
void delay_ms(UINT32 ms)
{
	while(ms--)
	{
		CLK_SysTickDelay(1000);
	}
}
