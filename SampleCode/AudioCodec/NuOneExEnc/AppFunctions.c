/*---------------------------------------------------------------------------------------------------------*/
/*																										   */
/* Copyright (c) Nuvoton Technology	Corp. All rights reserved.											   */
/*																										   */
/*---------------------------------------------------------------------------------------------------------*/

// ---------------------------------------------------------------------------------------------------------
//	Functions:
//		- Functions to handle main operations:
//			* Initiate application.
//			* Start audio playback.
//			* Stop  audio playback.
//			* Produce PCM data for audio playback.
//			* Start audio recording.
//			* Stop  audio recording.
//			* Use recorded data to do:
//				a. encoding
//				b. doing voice effect
//				c. write to storage
//				d. etc.
//		- The above functions use codes in "xxxApp" folder and "Utility" folder to complete necessary operations.
//
//	Reference "Readme.txt" for more information.
// ---------------------------------------------------------------------------------------------------------

#include "App.h"
#include "MicSpk.h"
#include "SPIFlashUtil.h"
# include "stdio.h"

extern UINT8 SPIFlash_Initiate(void);

extern void PowerDown_Enter(void);

extern void PowerDown(void);

extern void PowerDown_Exit(void);

extern S_APP g_sApp;

extern volatile UINT8 g_adcVR2volume;

extern volatile BOOL playEndStatus;

extern volatile BOOL recEndStatus;

extern volatile UINT8 g_u8AppCtrl;

extern UINT32 g_u32SPIFlashRecAddr;	// Current recoding address in SPIFlash.

extern S_SPIFLASH_HANDLER g_sSpiFlash; 

//---------------------------------------------------------------------------------------------------------
// Function: App_Initiate
//
// Description:
//	Initiate application.
//
// Argument:
//
// Return:
//
//---------------------------------------------------------------------------------------------------------
void App_Initiate(void)
{
	g_u8AppCtrl = APPCTRL_NO_ACTION;
	
	// Initiate the audio playback.
	Playback_Initiate();
	
	// Configure the UltraIO curve output funciton's interrupt priority.
	// Lower this priority if other interrupt services should be higher than UltraIO curve output function!
	#if ( ULTRAIO_FW_CURVE_ENABLE )
	NVIC_SetPriority(ULTRAIO_TMR_IRQ, 1);
	#endif	
	
	// Light stand by(PB8) led for initial ready().
	OUT3(0);
}

//---------------------------------------------------------------------------------------------------------
// Function: App_StartPlay
//
// Description:                                                                                           
//	Start audio playback.
//
// Return:
// 	FALSE: fail
//	TRUE:  success
//---------------------------------------------------------------------------------------------------------
BOOL App_StartPlay(void)
{	
	//this parameter is to prevent faulty when playing
	playEndStatus = FALSE;
	
	// Initiate NuOneEx audio decode lib with callback functions stored in g_asAppCallBack[0].
	NuOneExApp_DecodeInitiate(&g_sApp.sNuOneExAppDecode, (UINT8 *)&g_sApp.uTempBuf, 0);
	
	// Start NuOneEx decode lib to decode NuOneEx file stored from "sAudioChunkInfo.u32AudioChunkAddr"
	// And decode the first frame of PCMs.
	if ( NuOneExApp_DecodeStartPlayByAddr(&g_sApp.sNuOneExAppDecode, AUDIOROM_STORAGE_START_ADDR, 0) == 0)
		return FALSE;
	
	// Light playback led(PB4) for display status.
	OUT5(1);
	
	// Start Ultraio Timer & HW pwm for UltraIO curve output
	ULTRAIO_START();
	
	// Start to playback audio. 
	Playback_StartPlay();

	return TRUE;
}

//---------------------------------------------------------------------------------------------------------
// Description:                                                                                            
//	Stop audio playback.                                                                             
//
// Return:
// 	FALSE: fail
//	TRUE:  success
//---------------------------------------------------------------------------------------------------------
BOOL App_StopPlay(void)
{
	// Stop speaker.
	Playback_StopPlay();
	
	// Stop Ultraio Timer & HW pwm.
	ULTRAIO_STOP();

	OUT5(0);
	CLK_SysTickDelay(200000);  // Delay us
	OUT5(1);	

	//this parameter is to prevent faulty when playing
	playEndStatus = TRUE;
	
	return TRUE;
}

//---------------------------------------------------------------------------------------------------------
// Function: App_StartRec
//
// Description:                                                                                           
//	Record PCM data from MIC.
//
// Return:
// 	FALSE: fail
//	TRUE:  success
//---------------------------------------------------------------------------------------------------------
BOOL App_StartRec(void)
{	
	S_AUDIOCHUNK_HEADER sAudioChunkHeader;

	// Initiate NuOneEx audio encode lib with temp buffer provided for lib.
	NuOneExApp_EncodeInitiate(&g_sApp.sNuOneExAppEncode, (UINT8 *)&g_sApp.uTempBuf);
	
	// Start to encode NuOneEx data with sample rate, bit per frame and saved data information into audio chunk header.
	if (NuOneExApp_EncodeStart(&g_sApp.sNuOneExAppEncode, &sAudioChunkHeader, ADC_SAMPLE_RATE, E_NUONEEX_ENCODE_BPS_10) == FALSE)
		return FALSE;
	
	// SPIFlash utility function provide encode data write into SPIFlash.
	// detail info please refer "SPIFlashUtil.h"
	SPIFlahUtil_StartWriteEncodeData(&sAudioChunkHeader, AUDIOROM_STORAGE_START_ADDR, NULL);
	
	// Light record led for display status.
	OUT5(0);
	
	//this parameter is to prevent faulty when recording
	recEndStatus = FALSE;
	
	// Start to record PCM data into buffer for produc NuOneEx encode data.
	Record_StartRec();
	
	return TRUE;
}

//---------------------------------------------------------------------------------------------------------
// Description:                                                                                            
//	Stop to record PCM data.                                                                             
//
// Return:
// 	FALSE: fail
//	TRUE:  success
//---------------------------------------------------------------------------------------------------------
void App_StopRec(void)
{
	// Stop mic.
	Record_StopRec();
	
	// Stop to NuOneEx encode process.
	NuOneExApp_EncodeEnd(&g_sApp.sNuOneExAppEncode);
	
	// Write audio chunk header(encode data length) into SPIFlash.
	SPIFlashUtil_EndWriteEncodeData();

	OUT5(1);	

	//this parameter is to prevent faulty when recording
	recEndStatus = TRUE;		
}

//---------------------------------------------------------------------------------------------------------
// Function: App_ProcessPlay
//
// Description:                                                                                            
//   Produce PCM data for audio playback
//
// Return:
//	FALSE: No PCM produced for audio playback
//	TRUE:  Have PCMs produced for audio playback                                      
//---------------------------------------------------------------------------------------------------------
BOOL App_ProcessPlay(void)
{
	UINT8 u8ActiveProcessCount = 0;
	
	// Continue decode NuOneEx data to produce PCMs for audio playback.
	if ( NuOneExApp_DecodeProcess(&g_sApp.sNuOneExAppDecode) == TRUE )
		u8ActiveProcessCount ++;

	if ( u8ActiveProcessCount )
		return TRUE;

	return FALSE;
}

//---------------------------------------------------------------------------------------------------------
// Function: App_ProcessRec
//
// Description:                                                                                            
//   Record PCM data for providing Beat-detect.
//
// Return:
//	FALSE: No PCM produced for audio playback
//	TRUE:  Have PCMs produced for audio playback                                      
//---------------------------------------------------------------------------------------------------------
BOOL App_ProcessRec(void)
{
	// Write NuOneEx encode data into SPIFlash.
	SPIFlashUtil_WriteEncodeData(&g_sApp.sNuOneExAppEncode.sEncodeBufCtrl);

	// Check current record address is out size of SPIFlash.
	if (g_u32SPIFlashRecAddr >= g_sSpiFlash.u32FlashSize)
		return FALSE;	

	// Keep encode PCM buffer data to NuOneEx lib.
	NuOneExApp_EncodeProcess(&g_sApp.sNuOneExAppEncode);
	
	return TRUE;
}

//---------------------------------------------------------------------------------------------------------
// Function: App_PowerDown
//
// Description:                                                                                            
//   Process flow of power-down for application. Include,
//   1. App_PowerDownProcess:Pre-process befor entering power-down.
//   2. PowerDown:Power down base process(PowerDown.c).
//   3. App_WakeUpProcess:Post-process after exiting power-down.
//   User could disable or enable this flow from flag(POWERDOWN_ENABLE) in ConfigApp.h.
//---------------------------------------------------------------------------------------------------------
void App_PowerDown(void)
{
	App_StopPlay();
	App_StopRec();
	
	#if(POWERDOWN_ENABLE)
	PowerDown_Enter();
	PowerDown();
	PowerDown_Exit();
	#endif
}

//---------------------------------------------------------------------------------------------------------
// Function: App_ReadVR
//
// Description:                                                                                            
//   ADC read potentiometer and set speaker Volume
//---------------------------------------------------------------------------------------------------------
void App_ReadVRInit(void)
{
	uint32_t u32Div;
	
	/* Reset IP */
	CLK_EnableModuleClock(ADC_MODULE);
	CLK_EnableModuleClock(ANA_MODULE);
    SYS_ResetModule(EADC_RST);
	SYS_ResetModule(ANA_RST);
	
	/* Enable Analog block power */
	ADC_ENABLE_SIGNALPOWER(ADC,
	                       ADC_SIGCTL_ADCMOD_POWER|
						   ADC_SIGCTL_IBGEN_POWER|
	                       ADC_SIGCTL_BUFADC_POWER|
	                       ADC_SIGCTL_BUFPGA_POWER);
	
	/* PGA Setting */
	ADC_MUTEON_PGA(ADC, ADC_SIGCTL_MUTE_PGA);
	ADC_MUTEOFF_PGA(ADC, ADC_SIGCTL_MUTE_IPBOOST);

	ADC_ENABLE_PGA(ADC, ADC_PGACTL_REFSEL_VMID, ADC_PGACTL_BOSST_GAIN_0DB);
	ADC_SetPGAGaindB(0); // system default 600dB   


	/* MIC circuit configuration */
	ADC_ENABLE_VMID(ADC, ADC_VMID_HIRES_DISCONNECT, ADC_VMID_LORES_CONNECT);
	ADC_EnableMICBias(ADC_MICBSEL_90_VCCA);
	ADC_SetAMUX(ADC_MUXCTL_MIC_PATH, ADC_MUXCTL_POSINSEL_NONE, ADC_MUXCTL_NEGINSEL_NONE);
	
	/* Open ADC block */
	ADC_Open();
	ADC_SET_OSRATION(ADC, ADC_OSR_RATION_192);
	u32Div = CLK_GetHIRCFreq()/ADC_SAMPLE_RATE/192;
	ADC_SET_SDCLKDIV(ADC, u32Div);
	ADC_SET_FIFOINTLEVEL(ADC, 7);
	
	ADC_MUTEOFF_PGA(ADC, ADC_SIGCTL_MUTE_PGA);
	
}


void App_ReadVR()
{
    //uint32_t u32ConversionDatas[16];
	uint32_t u32ConversionData;
	uint8_t j;    
    
	printf("\n\n=== ADC single mode test ===\n");
	
	/* Init ADC */
	App_ReadVRInit();
				
    ADC_SetGPIOChannel(ADC_GPIO_SINGLEEND_CH5_N);
            
    // Enable ADC Interrupt function
    ADC_EnableInt(ADC_FIFO_INT);

    // Start A/D conversion 
    ADC_START_CONV(ADC);

    // Wait ADC interrupt 
    while(ADC_GetIntFlag(ADC_FIFO_INT));
			//while(1){
    for(j=0;j<=16;j++) {
        u32ConversionData = ADC_GET_FIFODATA(ADC);
        CLK_SysTickDelay(500);
    }
	
    g_adcVR2volume = u32ConversionData/4000;
    printf(" everage = (%ul)\n", g_adcVR2volume);
	
	// stop A/D conversion 
    ADC_STOP_CONV(ADC);
		
	CLK_SysTickDelay(100000);

	//Init MIC 

	//SYSCLK_INITIATE();				// Configure CPU clock source and operation clock frequency.
									// The configuration functions are in "SysClkConfig.h"
	
	CLK_EnableLDO(CLK_LDOSEL_3_3V);	// Enable ISD9100 interl 3.3 LDO.
	
//	if (! SPIFlash_Initiate())		// Initiate SPI interface and checking flows for accessing SPI flash.
//		while(1); 					// loop here for easy debug

	SPK_INITIATE();					// Initiate speaker including pop-sound canceling.
									// After initiation, the APU is paused.
									// Use SPK_Resume(0) to start APU operation.
									// Reference "MicSpk.h" for speaker related APIs.

	MIC_INITIATE();					// Initiate MIC.
									// After initiation, the ADC is paused.
									// Use ADC_Resume() to start ADC operation.
									// Reference "MicSpk.h" for MIC related APIs.
	
																	
	//App_Initiate();					// Initiate application for audio decode.			

//	Playback_SetVolumeDB(0, PLAYBACK_VOLUME_10_DB);   // added bylambor
	
}


