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

#define M 283 //elements in filter

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
#include "FIRCoeffs.txt"


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
double x[M];
int write_adr = 0;

 /******************************* Function prototypes ********************************/
void init_hardware(void);     
void init_HWI(void);

//Written in lab     
void ISR_AIC(void);          
void non_circ_addressing(void);
void circ_addressing(void);
void non_circ_symmetric_addressing(void);
void circ_symmetric_addressing(void);

/********************************** Main routine ************************************/
void main(){      
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
	//non_circ_addressing();
	//circ_addressing();
	//non_circ_symmetric_addressing();
	circ_symmetric_addressing();
}

void non_circ_symmetric_addressing(void)
{
	int halfM = floor(M/2);
	int odd = M % 2;
	short filtered = 0;
	int i;
	double var = mono_read_16Bit();
	
	for(i = M-1; i > 0; i--){
		x[i]=x[i-1];
	}
	
	x[0] = var;

	for(i = 0; i < halfM; i++){	
		filtered += (x[i] + x[M-i-1]) * filter[i];
	}
	if(odd){
		filtered += x[halfM] * filter[halfM];
	}
	
	mono_write_16Bit(filtered);
}
void circ_symmetric_addressing(void)
{
	int halfM = floor(M/2);
	int odd = M % 2;
	double filtered = 0;
	int offset1, offset2, i;
	
	offset1 = write_adr;
	offset2 = write_adr;
	
	x[write_adr] = mono_read_16Bit();
	
	
	for(i = 0; i < halfM; i++){	
		int adr1 = offset1 - i;
		int adr2 = offset2 + 1 + i;
		
		if(adr1 < 0){
			offset1 += M;
			adr1 = offset1 - i;
		}
		if(adr2 >= M){
			offset2 -= M;
			adr2 = offset2 + 1 + i;
		}
		
		filtered += (x[adr1] + x[adr2]) * filter[i]; 
	}
	
	
	if(odd){
		offset1 = write_adr - halfM;
		if(offset1 < 0){
			filtered += x[offset1 + M] * filter[halfM];
		}else{
			filtered += x[offset1] * filter[halfM];
		}
	}
	
	mono_write_16Bit(filtered);
	
	write_adr++;
	if(write_adr == M){
		write_adr = 0;
	}
	
}

void circ_addressing(void)
{
	short filtered = 0;
	int offset, i;
	
	write_adr++;
	write_adr = write_adr % M;
	offset = write_adr;
	x[write_adr] = mono_read_16Bit();
	
	for(i = 0; i < M; i++){
		int temp = i + offset;
		if(temp >= M){
			offset -= M;
			temp = i + offset;
		}
		filtered += x[temp] * filter[i];
	}
	
	mono_write_16Bit(filtered);
	
	
}

void non_circ_addressing(void){
	double filtered = 0;
	int i;
	double val = mono_read_16Bit();
	for(i = M-1; i > 0; i--){
		x[i]=x[i-1];
	}
	
	x[0] = val; 
	
	for(i = 0; i < M; i++){
		filtered += x[i] * filter[i];
	}
	
	mono_write_16Bit(filtered);
	
}
