/*************************************************************************************
			       DEPARTMENT OF ELECTRICAL AND ELECTRONIC ENGINEERING
					   		     IMPERIAL COLLEGE LONDON 

 				      EE 3.19: Real Time Digital Signal Processing
					       Dr Paul Mitcheson and Daniel Harvey

				        		  LAB 3: Interrupt I/O

 				            ********* I N T I O. C **********

  Demonstrates inputing and outputing data from the DSK's audio port using interrupts. 

 *************************************************************************************
 				Updated for use on 6713 DSK by Danny Harvey: May-Aug 2006
				Updated for CCS V4 Sept 10
 ************************************************************************************/
/*
 *	You should modify the code so that interrupts are used to service the 
 *  audio port.
 */
/**************************** Pre-processor statements ******************************/

#define SINE_TABLE_SIZE 256
#define PI 3.141592653589793

#include <stdlib.h>
//  Included so program can make use of DSP/BIOS configuration tool.  
#include "dsp_bios_cfg.h"

/* The file dsk6713.h must be included in every program that uses the BSL.  This 
   example also includes dsk6713_aic23.h because it uses the 
   AIC23 codec module (audio interface). */
#include "dsk6713.h"
#include "dsk6713_aic23.h"

// math library (trig functions)
#include <math.h>

// Some functions to help with writing/reading the audio ports when using interrupts.
#include <helper_functions_ISR.h>

/******************************* Global declarations ********************************/

/* Audio port configuration settings: these values set registers in the AIC23 audio 
   interface to configure it. See TI doc SLWS106D 3-3 to 3-10 for more info. */
DSK6713_AIC23_Config Config = { \
			 /**********************************************************************/
			 /*   REGISTER	            FUNCTION			      SETTINGS         */ 
			 /**********************************************************************/\
    0x0017,  /* 0 LEFTINVOL  Left line input channel volume  0dB                   */\
    0x0017,  /* 1 RIGHTINVOL Right line input channel volume 0dB                   */\
    0x01f9,  /* 2 LEFTHPVOL  Left channel headphone volume   0dB                   */\
    0x01f9,  /* 3 RIGHTHPVOL Right channel headphone volume  0dB                   */\
    0x0011,  /* 4 ANAPATH    Analog audio path control       DAC on, Mic boost 20dB*/\
    0x0000,  /* 5 DIGPATH    Digital audio path control      All Filters off       */\
    0x0000,  /* 6 DPOWERDOWN Power down control              All Hardware on       */\
    0x0043,  /* 7 DIGIF      Digital audio interface format  16 bit                */\
    0x008d,  /* 8 SAMPLERATE Sample rate control             8 KHZ                 */\
    0x0001   /* 9 DIGACT     Digital interface activation    On                    */\
			 /**********************************************************************/
};


// Codec handle:- a variable used to identify audio interface  
DSK6713_AIC23_CodecHandle H_Codec;

float table[SINE_TABLE_SIZE]; //The array that contains the lookup table values
float index = 0; //Used for tracking the current index of the lookup table
/* Use this variable in your code to set the frequency of your sine wave 
   be carefull that you do not set it above the current nyquist frequency! */
float sine_freq = 1000.0;


 /******************************* Function prototypes ********************************/
void init_hardware(void);     
void init_HWI(void);
//Written in lab     
void ISR_AIC(void);          
void sine_init(void);    
float sinegen(void);
/********************************** Main routine ************************************/
void main(){      

 
	// initialize board and the audio port
  init_hardware();
	
  /* initialize hardware interrupts */
  init_HWI();
  sine_init();
  /* loop indefinitely, waiting for interrupts */  					
  while(1) 
  {};
  
}
        
/********************************** init_hardware() **********************************/  
void init_hardware()
{
    // Initialize the board support library, must be called first 
    DSK6713_init();
    
    // Start the AIC23 codec using the settings defined above in config 
    H_Codec = DSK6713_AIC23_openCodec(0, &Config);

	/* Function below sets the number of bits in word used by MSBSP (serial port) for 
	receives from AIC23 (audio port). We are using a 32 bit packet containing two 
	16 bit numbers hence 32BIT is set for  receive */
	MCBSP_FSETS(RCR1, RWDLEN1, 32BIT);	

	/* Configures interrupt to activate on each consecutive available 32 bits 
	from Audio port hence an interrupt is generated for each L & R sample pair */	
	MCBSP_FSETS(SPCR1, RINTM, FRM);

	/* These commands do the same thing as above but applied to data transfers to  
	the audio port */
	MCBSP_FSETS(XCR1, XWDLEN1, 32BIT);	
	MCBSP_FSETS(SPCR1, XINTM, FRM);	
	

}

/********************************** init_HWI() **************************************/  
void init_HWI(void)
{
	IRQ_globalDisable();			// Globally disables interrupts
	IRQ_nmiEnable();				// Enables the NMI interrupt (used by the debugger)
	/*
	IRQ_map(IRQ_EVT_RINT1,4);		// Maps an event to a physical interrupt
	IRQ_enable(IRQ_EVT_RINT1);		// Enables the event
	*/
	IRQ_map(IRQ_EVT_XINT1,4);		// Maps an event to a physical interrupt
	IRQ_enable(IRQ_EVT_XINT1);		// Enables the event
	IRQ_globalEnable();				// Globally enables interrupts

} 

/******************** WRITE YOUR INTERRUPT SERVICE ROUTINE HERE***********************/  


void ISR_AIC(void)
{
	/*	
	short sample;
	sample = mono_read_16Bit();
	
	if(sample < 0){
		sample = sample * -1;
	}
	
	mono_write_16Bit(sample);
	*/
	short sample;
	sample = sinegen() * 32000;
	if(sample > 0){
		sample = sample * -1;
	}
	mono_write_16Bit(sample);
	
}

float sinegen(void)
{	
	/*
	The function calculates the increment required for the index of the lookup table (fractionally) for a given sample rate and desired frequency
	This index is then rounded down to ensure that it is valid for lookup
	The corresponding look up value is then returned
	*/
	
	float table_sample;
	float increment;
	int index_round;
	int sampling_freq = 8000;
	
	index_round = (int) round(index); //the current index is rounded to a valid index
	table_sample = table[index_round]; //the required sample is recovered from the lookup table
	
	increment = (float) SINE_TABLE_SIZE * sine_freq / (float) sampling_freq; //the increment value of the index is calculated
	
	index = index + increment; //the index is incremented 
	
	//if index is equal or greater to 256 it must be wrapped around
	while(index >= SINE_TABLE_SIZE){
		index = index - SINE_TABLE_SIZE;
	}
	
	return table_sample;
}




/*we added this*/
void sine_init(){
	int i;
	for(i = 0; i < SINE_TABLE_SIZE; i++){
		double sin_arg = (double)i / (double)SINE_TABLE_SIZE;
		double sin_val = sin(2 * PI * sin_arg);
		table[i] = sin_val;
	
	}
}















