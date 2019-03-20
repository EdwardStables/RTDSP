/*************************************************************************************
			       DEPARTMENT OF ELECTRICAL AND ELECTRONIC ENGINEERING
					   		     IMPERIAL COLLEGE LONDON 

 				      EE 3.19: Real Time Digital Signal Processing
					       Dr Paul Mitcheson and Daniel Harvey

				        		 PROJECT: Frame Processing

 				            ********* ENHANCE. C **********
							 Shell for speech enhancement 

  		Demonstrates overlap-add frame processing (interrupt driven) on the DSK. 

 *************************************************************************************
 				             By Danny Harvey: 21 July 2006
							 Updated for use on CCS v4 Sept 2010
**************************************************************************************
						Noise removal algorithm implemented by
					   Edward Stables and Jack Waller March 2019
 ************************************************************************************/

 
/**************************** Pre-processor statements ******************************/
//  library required when using calloc
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

/* Some functions to help with Complex algebra and FFT. */
#include "cmplx.h"      
#include "fft_functions.h"  

// Some functions to help with writing/reading the audio ports when using interrupts.
#include <helper_functions_ISR.h>

#define WINCONST 0.85185			/* 0.46/0.54 for Hamming window */
#define FSAMP 8000.0				 /* sample frequency, ensure this matches Config for AIC */
#define FFTLEN 256					/* fft length = frame length 256/8000 = 32 ms*/
#define NFREQ (1+FFTLEN/2)			/* number of frequency bins from a real FFT */
#define OVERSAMP 4					/* oversampling ratio (2 or 4) */  
#define FRAMEINC (FFTLEN/OVERSAMP)	/* Frame increment */
#define CIRCBUF (FFTLEN+FRAMEINC)	/* length of I/O buffers */

#define OUTGAIN 16000.0				/* Output gain for DAC */
#define INGAIN  (1.0/16000.0)		/* Input gain for ADC  */
// PI defined here for use in your code 
#define PI 3.141592653589793
#define TFRAME FRAMEINC/FSAMP       /* time between calculation of each frame */





/* Switches */
int method = 6;
int filtering = 1;
int dynamic_alpha = 1;
int residual_noise_reduction = 1;

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
    0x008d,  /* 8 SAMPLERATE Sample rate control        8 KHZ-ensure matches FSAMP */\
    0x0001   /* 9 DIGACT     Digital interface activation    On                    */\
			 /**********************************************************************/
};


DSK6713_AIC23_CodecHandle H_Codec;						/* Codec handle:- a variable used to identify audio interface */

float *inbuffer, *outbuffer;   							/* Input/output circular buffers */
complex *fftframe;          							/* Input and output frames, complex for FFT operation */
float *inwin, *outwin;              					/* Input and output windows */
float ingain, outgain;									/* ADC and DAC gains */ 
float cpufrac; 											/* Fraction of CPU time used */
volatile int io_ptr=0;              					/* Input/ouput pointer for circular buffers */
volatile int frame_ptr=0;           					/* Frame pointer */

float *M1, *M2, *M3, *M4;								/* Minimum noise spectrum estimation frames */
float *smoothed_samples;								/* Input spectrum processed with smoothing */
complex *n_minus_one, *n_minus_two, *n_minus_zero;		/* Two previous and one current output frame for residual noise reduction */
int *exceed_n_minus_one, *exceed_n_minus_zero;			/* Arrays that signal that residual reduction limit has been reached */
int frame_counter = 0;									/* Tracks number of frames processed signal a buffer rotation */


/******************************* Algorithm Variables *******************************/
float LAMBDA = 0.0075;									/* Scaling constant for the noise floor */
float ALPHA = 8;										/* Variable scaling constant for noise overestimation */
float ALPHA_0 = 12.4;									/* Offset value for selection of ALPHA */
float smoothing_const = 0.855;							/* Constant used in low pass filtering of input signal */
float dynamic_slope = 13/25;							/* Gradiant of variable ALPHA */	
float residual_reduction_threshold = 1.1;				/* Threshold that must be exceeded for residual reduction to be applied */
float sum_threshold = 0.5;								/* Threshold that noise buffers must exceed to not be set to the maximum float value */

/******************************* Function prototypes *******************************/
void init_hardware(void);    							/* Initialize codec */ 
void init_HWI(void);            						/* Initialize hardware interrupts */
void ISR_AIC(void);             						/* Interrupt service routine for codec */
void process_frame(void);       						/* Frame processing routine */
           
/********************************** Main routine ************************************/
void main()
{      

  	int k; // used in various for loops
  
	/*  Initialize and zero fill arrays */  	
	inbuffer	= (float *) calloc(CIRCBUF, sizeof(float));		/* Input array */
    outbuffer	= (float *) calloc(CIRCBUF, sizeof(float));		/* Output array */
    fftframe	= (complex *) calloc(FFTLEN, sizeof(complex));	/* Array for processing, complex for fft*/
    inwin		= (float *) calloc(FFTLEN, sizeof(float));		/* Input window */
    outwin		= (float *) calloc(FFTLEN, sizeof(float));		/* Output window */
    
    M1			= (float *) calloc(FFTLEN, sizeof(float));
    M2			= (float *) calloc(FFTLEN, sizeof(float));
    M3			= (float *) calloc(FFTLEN, sizeof(float));
    M4			= (float *) calloc(FFTLEN, sizeof(float));
	
	smoothed_samples 	= (float *) calloc(FFTLEN, sizeof(float));
	n_minus_zero 		= (complex *) calloc(FFTLEN, sizeof(complex));
	n_minus_one 		= (complex *) calloc(FFTLEN, sizeof(complex));
	n_minus_two 		= (complex *) calloc(FFTLEN, sizeof(complex));
	exceed_n_minus_zero = (int *) calloc(FFTLEN, sizeof(int));
	exceed_n_minus_one 	= (int *) calloc(FFTLEN, sizeof(int));
	
	/* initialize board and the audio port */
  	init_hardware();
  
  	/* initialize hardware interrupts */
  	init_HWI();    
  

                       
  	for (k=0;k<FFTLEN;k++)
	{                           
		inwin[k] = sqrt((1.0-WINCONST*cos(PI*(2*k+1)/FFTLEN))/OVERSAMP);
		outwin[k] = inwin[k]; 
		M1[k] = LAMBDA; 
		M2[k] = LAMBDA; 
		M3[k] = LAMBDA; 
		M4[k] = LAMBDA;
	} 
	
  	ingain=INGAIN;
  	outgain=OUTGAIN;        

 							
  	/* main loop, wait for interrupt */  
  	while(1) 	process_frame();
}
    
/********************************** init_hardware() *********************************/  
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

	/* These commands do the same thing as above but applied to data transfers to the 
	audio port */
	MCBSP_FSETS(XCR1, XWDLEN1, 32BIT);	
	MCBSP_FSETS(SPCR1, XINTM, FRM);	
	

}
/********************************** init_HWI() **************************************/ 
void init_HWI(void)
{
	IRQ_globalDisable();			// Globally disables interrupts
	IRQ_nmiEnable();				// Enables the NMI interrupt (used by the debugger)
	IRQ_map(IRQ_EVT_RINT1,4);		// Maps an event to a physical interrupt
	IRQ_enable(IRQ_EVT_RINT1);		// Enables the event
	IRQ_globalEnable();				// Globally enables interrupts

}
        
/******************************** process_frame() ***********************************/  
void process_frame(void)
{
	int k, m; 
	int io_ptr0;   
	float *min_noise;
	float M1_sum = 0.0;
	float M2_sum = 0.0;
	float M3_sum = 0.0;
	float M4_sum = 0.0;
	 
	complex *p;
	int *q;
	
	float signal_power_sum = 0;
	float noise_power_sum = 0;
	float SNR;
	
	/* work out fraction of available CPU time used by algorithm */    
	cpufrac = ((float) (io_ptr & (FRAMEINC - 1)))/FRAMEINC;  
		
	/* wait until io_ptr is at the start of the current frame */ 	
	while((io_ptr/FRAMEINC) != frame_ptr); 
	
	/* then increment the framecount (wrapping if required) */ 
	if (++frame_ptr >= (CIRCBUF/FRAMEINC)) frame_ptr=0;
 	
 	/* save a pointer to the position in the I/O buffers (inbuffer/outbuffer) where the 
 	data should be read (inbuffer) and saved (outbuffer) for the purpose of processing */
 	io_ptr0=frame_ptr * FRAMEINC;
	
	/* copy input data from inbuffer into inframe (starting from the pointer position) */ 
	 
	m=io_ptr0;
    for (k=0;k<FFTLEN;k++)
	{                           
		fftframe[k].r = inbuffer[m] * inwin[k]; 
		fftframe[k].i = 0;
		if (++m >= CIRCBUF) m=0; /* wrap if required */
	} 
	
	/************************* DO PROCESSING OF FRAME  HERE **************************/
	
	
	
	
	
	fft(FFTLEN, fftframe);					      	
	
	/* Pre filtering */
	for(k = 0; k < FFTLEN; k++)
		smoothed_samples[k] = ((1-smoothing_const) * cabs(fftframe[k])) + (smoothing_const * smoothed_samples[k]);

	/* Update minimum noise buffers */
	if(++frame_counter >= 78){
		//Rotate buffers if 78 frames have been processed (32ms * 78 = 2.496 seconds)
		float *p;
		p = M4;
		M4 = M3;
		M3 = M2;
		M2 = M1;
		M1 = p;
		for (k=0; k < FFTLEN; k++)
		{
			//Fill M1 with the entirity of current input frame
			M1[k] = smoothed_samples[k];
		}
		frame_counter = 0;
	}else{			
		//If current input's bins contain a lower frequency value than that currently stored then replace that value
	    for (k=0;k<FFTLEN;k++)
		{                 
			if(val < M1[k])	M1[k] = smoothed_samples[k];
		} 
	}
	
	/* Select minimum noise buffer */
	
	//Take squared sum of each buffer
	for(k = 0; k < FFTLEN; k++){
		M1_sum += M1[k] * M1[k];
		M2_sum += M2[k] * M2[k];
		M3_sum += M3[k] * M3[k];
		M4_sum += M4[k] * M4[k];
	}
	
	//Set frame to a maximum if it is valued at below the threshold (see report for reasoning)
	if(abs(M2_sum) < sum_threshold) M2_sum = FLT_MAX;
	if(abs(M3_sum) < sum_threshold) M3_sum = FLT_MAX;
	if(abs(M4_sum) < sum_threshold) M4_sum = FLT_MAX;
	
	//Select minimum noise values
	if(M1_sum < M2_sum && M1_sum < M3_sum && M1_sum < M4_sum) min_noise = M1;
	else if(M2_sum < M3_sum && M2_sum < M4_sum)	min_noise = M2;
	else if(M3_sum < M4_sum) min_noise = M3;
	else min_noise = M4;
		
	
	/* Determine dynamic alpha */
	for(k = 0; k < FFTLEN; k++){
		//Determine power of current signal and noise estimation
		signal_power_sum += smoothed_samples[k] * smoothed_samples[k];
		noise_power_sum += min_noise[k] * min_noise[k];
	}
	//Calcuate SNR estimate
	SNR = 10 * log10f(signal_power_sum / noise_power_sum);
	
	//Set alpha 
	if(SNR > 20){
		ALPHA = 2;
	}else if(SNR < -5){
		ALPHA = 15;
	}else{
		ALPHA = ALPHA_0 - (SNR * dynamic_slope);
	}
		
	
	
	/* Perform noise subtratction */
		
	for(k=0; k < FFTLEN; k++){
		//Determine power of frequency bin in sample and noise estimation
		float signal_power = smoothed_samples[k] * smoothed_samples[k];
		float noise_power = min_noise[k] * min_noise[k];
		
		//Determine two possible values of output 
		float subtraction = signal_power - (ALPHA * noise_power);
		float noise_floor = LAMBDA * noise_power;
		
		//Take maximum of the subtraction or noise_floor
		if(subtraction > noise_floor){
			//If subtraction is max, set output coefficent as its square root (moving back to magnitude)
			float coef = sqrt(1 - (ALPHA * noise_power / signal_power)); 
			
			//Save values in n_minus_zero[] to perform residual noise reduction later
			n_minus_zero[k].r = fftframe[k].r * coef;
			n_minus_zero[k].i = fftframe[k].i * coef;
			
			//Determine if a sample might need replacing by checking if it exceeds the threshold
			if(noise_power/signal_power > residual_reduction_threshold){
				exceed_n_minus_zero[k] = 1;
			}else{
				exceed_n_minus_zero[k] = 0;
			}

		}else{
			//If the noise_floor is greater than the sample then the residual noise reduction is not required, therefore output just set
			noise_floor = sqrt(noise_floor);
			exceed_n_minus_zero[k] = 0;
			n_minus_zero[k].r = fftframe[k].r * noise_floor;
			n_minus_zero[k].i = fftframe[k].i * noise_floor;
			
		
		}
	
	}
	
	/* Residual Noise Reduction*/
	
	for(k = 0; k < FFTLEN; k++){
		//Only perform if the sample exceeded the limit
		if(exceed_n_minus_one[k] == 1){
			//Take magnitude of sample from current output frame, and its adjacent frames
			float mag_0 = cabs(n_minus_zero[k]);
			float mag_1 = cabs(n_minus_one[k]);
			float mag_2 = cabs(n_minus_two[k]);
			//Select minimum value of the three samples
			if(mag_0 < mag_1 && mag_0 < mag_2){
				fftframe[k] = n_minus_zero[k];
			}else if(mag_1 < mag_2){
				fftframe[k] = n_minus_one[k];
			}else{
				fftframe[k] = n_minus_two[k];
			}
		}else{
			fftframe[k] = n_minus_one[k];
		}
	}
	//Perform buffer rotations
	p = n_minus_two;
	n_minus_two = n_minus_one;
	n_minus_one = n_minus_zero;
	n_minus_zero = p;
	
	q = exceed_n_minus_one;
	exceed_n_minus_one = exceed_n_minus_zero;
	exceed_n_minus_zero = q; 
	
	ifft(FFTLEN, fftframe);
	
	/********************************************************************************/
	
    /* multiply outframe by output window and overlap-add into output buffer */  
                           
	m=io_ptr0;
    
    for (k=0;k<(FFTLEN-FRAMEINC);k++) 
	{    										/* this loop adds into outbuffer */                       
	  	outbuffer[m] = outbuffer[m]+fftframe[k].r*outwin[k];   
		if (++m >= CIRCBUF) m=0; /* wrap if required */
	}         
    for (;k<FFTLEN;k++) 
	{                           
		outbuffer[m] = fftframe[k].r*outwin[k];   /* this loop over-writes outbuffer */        
	    m++;
	}	                                   
}        
/*************************** INTERRUPT SERVICE ROUTINE  *****************************/

// Map this to the appropriate interrupt in the CDB file
   
void ISR_AIC(void)
{       
	short sample;
	/* Read and write the ADC and DAC using inbuffer and outbuffer */
	
	sample = mono_read_16Bit();
	inbuffer[io_ptr] = ((float)sample)*ingain;
		/* write new output data */
	mono_write_16Bit((int)(outbuffer[io_ptr]*outgain)); 
	
	/* update io_ptr and check for buffer wraparound */    
	
	if (++io_ptr >= CIRCBUF) io_ptr=0;
}

/************************************************************************************/
