/*************************************************************************************
			       DEPARTMENT OF ELECTRICAL AND ELECTRONIC ENGINEERING
					   		     IMPERIAL COLLEGE LONDON 

 				      EE 3.19: Real Time Digital Signal Processing
					       Dr Paul Mitcheson and Daniel Harvey

				        		  LAB 4: FIR Filters 

 *************************************************************************************
 				Updated for use on 6713 DSK by Danny Harvey: May-Aug 2006
				Updated for CCS V4 Sept 10
**************************** Pre-processor statements ******************************/
#include <stdlib.h>
// math library (trig functions)
#include <math.h>
//  Included so program can make use of DSP/BIOS configuration tool.  
#include "dsp_bios_cfg.h"

/* The file dsk6713.h must be included in every program that uses the BSL.  This 
   example also includes dsk6713_aic23.h because it uses the 
   AIC23 codec module (audio interface). */
#include "dsk6713.h"
#include "dsk6713_aic23.h"
#include "2_coef.txt"
//#include "4_coef.txt"
//#include "6_coef.txt"
//#include "8_coef.txt"
//#include "10_coef.txt"

#define N (sizeof(a)/sizeof(a[0]) - 1)

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

//Global variables for value tracking and removing repeated calculations

//Tustin IIR filter
double prev_output_scale = 0;
double prev_input = 0;


//Direct form II
//int N = sizeof(a)/sizeof(a[0]) - 1;
double delay_line[N];  
//double delay_line[] = {0, 0, 0, 0};
 /******************************* Function prototypes ********************************/
void init_hardware(void);     
void init_HWI(void);

//Written in lab     
void ISR_AIC(void);          
double RC_IIR(double sample);
double direct_form_2(double sample);
double direct_form_2_transposed(double sample);

/********************************** Main routine ************************************/
void main(){      
  int i;
  
  for(i = 0; i < N; i++){
  	delay_line[i] = 0;
  }
  
  // initialize board and the audio port
  init_hardware();
	
  /* initialize hardware interrupts */
  init_HWI();
  
  
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
	double sample, output;
	
	sample = mono_read_16Bit();
	
	output = RC_IIR(sample);
	//output = direct_form_2(sample);
	//output = direct_form_2_transposed(sample);
	
	mono_write_16Bit(output);
}

double direct_form_2_transposed(double x){
	double y, var;
	int i;
	
	y = (x * b[0]) + delay_line[N-1];
	
	for(i = N-1; i > 0; i--)
	{
		delay_line[i] = delay_line[i-1];
		var = (b[N-i] * x) - (a[N-i] * y);
		delay_line[i] += var;
	}
	delay_line[0] = (b[N] * x) - (a[N] * y);
	return y;
}

double direct_form_2(double x){
	
	double y, var;//,x;
	int i;
	
	//x = sample;
	for(i = 0; i < N; i++)
	{	
		var = delay_line[i] * a[i+1];
		x -= var;
	}
	
	y = x * b[0];
	
	for(i = 0; i < N; i++)
	{
		var = delay_line[i] * b[i+1]; 
		y += var;
	}
	
	
	for(i = N - 1; i > 0; i--)
	{
		delay_line[i] = delay_line[i-1];
	}
	
	delay_line[0] = x;
	return y;
	
}

double RC_IIR(double input)
{
	/*
	 * implements: y(n) = (x(n) + x(n-1) +15y(n-1))/17
	 * RC network with R = 1k, C = 1u
	 * Measured 5RC = 6.5ms -> RC = 1.3ms
	 */
	double output;
		
	output = input + prev_input + prev_output_scale;
	prev_input = input;
	output = output / 17;
	
	prev_output_scale = 15 * output;
	
	return output;
}
