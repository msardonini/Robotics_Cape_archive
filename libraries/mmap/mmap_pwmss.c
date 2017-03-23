/*
Copyright (c) 2014, James Strawson
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer. 
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies, 
either expressed or implied, of the FreeBSD Project.
*/

//#define DEBUG

#include "mmap_pwmss.h"
#include "tipwmss.h"

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdint.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>

volatile char *cm_per_base;
int cm_per_mapped=0;
volatile char *pwm_base[3]; // pwm subsystem pointers for eQEP
int pwmss_mapped[3] = {0,0,0}; // to record which subsystems have been mapped
int eqep_initialized[3] = {0,0,0};
int pwm_initialized[3] = {0,0,0};

/********************************************
*  PWMSS Mapping
*********************************************/
// maps the base of each PWM subsystem into an array
// this is used by eQEP and PWM
// returns immediately if this has already been done 
int map_pwmss(int ss){
	if(ss>2 || ss<0){
		printf("error: PWM subsystem must be 0, 1, or 2\n");
		return -1;
	}
	//return 0 if it's already been mapped.
	if(pwmss_mapped[ss]){
		return 0;
	}
	
	//open /dev/mem file pointer for mmap
	#ifdef DEBUG
		printf("opening /dev/mem\n");
	#endif
	int dev_mem;
	if ((dev_mem = open("/dev/mem", O_RDWR | O_SYNC))==-1){
	  printf("Could not open /dev/mem \n");
	  return -1;
	}
	
	// first open the clock register to see if the PWMSS has clock enabled
	if(!cm_per_mapped){
		#ifdef DEBUG
		printf("mapping CM_PER\n");
		#endif
		cm_per_base=mmap(0,CM_PER_PAGE_SIZE,PROT_READ|PROT_WRITE,MAP_SHARED,dev_mem,CM_PER);
		if(cm_per_base == (void *) -1) {
			printf("Unable to mmap cm_per\n");
			return -1;
		}
		cm_per_mapped = 1;
	}
	
	// if this subsystem hasn't already been mapped, 
	// then we probably need to enable clock signal to it in cm_per
	uint32_t cm_per_clkctrl;
	switch(ss){
	case 0:
		cm_per_clkctrl = CM_PER_EPWMSS0_CLKCTRL;
		break;
	case 1:
		cm_per_clkctrl = CM_PER_EPWMSS1_CLKCTRL;
		break;
	case 2:
		cm_per_clkctrl = CM_PER_EPWMSS2_CLKCTRL;
		break;
	default:
		return -1;
	}
	
	*(uint16_t*)(cm_per_base + cm_per_clkctrl) |= MODULEMODE_ENABLE;
	#ifdef DEBUG
	printf("new clkctrl%d: %d\n", ss, *(uint16_t*)(cm_per_base + cm_per_clkctrl));
	#endif

	
	// now map the appropriate subsystem base address
	#ifdef DEBUG
		printf("calling mmap() for base %d\n", ss);
	#endif
	switch(ss){
	case 0:
		pwm_base[0] = mmap(0,PWMSS_MEM_SIZE,PROT_READ|PROT_WRITE,MAP_SHARED,dev_mem,PWMSS0_BASE);
		break;
	case 1:
		pwm_base[1] = mmap(0,PWMSS_MEM_SIZE,PROT_READ|PROT_WRITE,MAP_SHARED,dev_mem,PWMSS1_BASE);
		break;
	case 2:
		pwm_base[2] = mmap(0,PWMSS_MEM_SIZE,PROT_READ|PROT_WRITE,MAP_SHARED,dev_mem,PWMSS2_BASE);
		break;
	default:
		printf("invalid ss\n");
		return -1;
	}
	#ifdef DEBUG
		printf("finished mapping for base %d\n", ss);
	#endif
	
	if(pwm_base[ss] == (void *) -1) {
		printf("Unable to mmap pwm \n");
		return -1;
	}
	pwmss_mapped[ss]=1;
	
	// enable clock from PWMSS
	*(uint32_t*)(pwm_base[ss]+PWMSS_CLKCONFIG) |= 0x010;
	
	close(dev_mem);
	#ifdef DEBUG
		printf("closed /dev/mem\n");
	#endif
	return 0;
}

/********************************************
*  eQEP
*********************************************/

// init_eqep takes care of sanity checks and returns quickly
// if nothing is to be initialized.
int init_eqep(int ss){
	// range sanity check
	if(ss>2 || ss<0){
		printf("error: PWM subsystem must be 0, 1, or 2\n");
		return -1;
	}
	// see if eQEP already got initialized
	if(eqep_initialized[ss]){
		return 0;
	}
	// make sure the subsystem is mapped
	if(map_pwmss(ss)){
		printf("failed to map PWMSS %d\n", ss);
		return -1;
	}
	#ifdef DEBUG
		printf("setting eqep ctrl registers\n");
	#endif
	//turn off clock to eqep
	*(uint32_t*)(pwm_base[ss]+PWMSS_CLKCONFIG) &= ~PWMSS_EQEPCLK_EN;
	// Write the decoder control settings
	*(uint16_t*)(pwm_base[ss]+EQEP_OFFSET+QDECCTL) = 0;
	// set maximum position to two's compliment of -1, aka UINT_MAX
	*(uint32_t*)(pwm_base[ss]+EQEP_OFFSET+QPOSMAX)=-1;
	// Enable interrupt
	*(uint16_t*)(pwm_base[ss]+EQEP_OFFSET+QEINT) = UTOF;
	// set unit period register
	*(uint32_t*)(pwm_base[ss]+EQEP_OFFSET+QUPRD)=0x5F5E100;
	// enable counter in control register
	*(uint16_t*)(pwm_base[ss]+EQEP_OFFSET+QEPCTL) = PHEN|IEL0|SWI|UTE|QCLM;
	//enable clock from PWMSS
	*(uint32_t*)(pwm_base[ss]+PWMSS_CLKCONFIG) |= PWMSS_EQEPCLK_EN;
	
	// Test eqep by resetting position
	#ifdef DEBUG
		printf("testing eQEP write\n");
	#endif
	*(uint32_t*)(pwm_base[ss] + EQEP_OFFSET +QPOSCNT) = 0;
	#ifdef DEBUG
		printf("successfully tested eQEP write\n");
	#endif
	eqep_initialized[ss] = 1;
	return 0;
}

// read a value from eQEP counter
int read_eqep(int ch){
	if(init_eqep(ch)) return -1;
	return  *(int*)(pwm_base[ch] + EQEP_OFFSET +QPOSCNT);
}

// write a value to the eQEP counter
int write_eqep(int ch, int val){
	if(init_eqep(ch)) return -1;
	*(int*)(pwm_base[ch] + EQEP_OFFSET +QPOSCNT) = val;
	return 0;
}

/****************************************************************
* PWM
* Due to conflicts with the linux driver the only available
* mmap function for PWM is to set the duty cycle. Setup
* must be done through /sys/class/pwm or with simple_pwm.c
*****************************************************************/

// set duty cycle for either channel A or B in a given subsystem
// input channel is a character 'A' or 'B'
int set_pwm_duty(int ss, char ch, float duty){
	// make sure the subsystem is mapped
	if(map_pwmss(ss)){
		printf("failed to map PWMSS %d\n", ss);
		return -1;
	}
	//sanity check duty
	if(duty>1.0 || duty<0.0){
		printf("duty must be between 0.0 & 1.0\n");
		return -1;
	}
	
	// duty ranges from 0 to TBPRD+1 for 0-100% PWM duty
	uint16_t period = *(uint16_t*)(pwm_base[ss]+PWM_OFFSET+TBPRD);
	uint16_t new_duty = (uint16_t)lroundf(duty * (period+1));
	
	#ifdef DEBUG
		printf("period : %d\n", period);
		printf("new_duty : %d\n", new_duty);
	#endif
	
	// change appropriate compare register
	switch(ch){
	case 'A':
		*(uint16_t*)(pwm_base[ss]+PWM_OFFSET+CMPA) = new_duty;
		break;
	case 'B':
		*(uint16_t*)(pwm_base[ss]+PWM_OFFSET+CMPB) = new_duty;
		break;
	default:
		printf("pwm channel must be 'A' or 'B'\n");
		return -1;
	}
	
	return 0;
}
