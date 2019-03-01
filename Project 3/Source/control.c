#include <MKL25Z4.H>
#include <stdio.h>
#include <stdint.h>

#include "gpio_defs.h"
#include "debug.h"

#include "control.h"

#include "timers.h"
#include "delay.h"
#include "LEDS.h"

#include "HBLED.h"
#include "FX.h"

#include "MMA8451.h"
#include "my_data.h"

extern osMessageQueueId_t ADC_Server;
volatile uint16_t *pointer_to_result;

volatile int g_enable_control=1;
volatile int g_set_current=DEF_LED_CURRENT_MA; // Default starting LED current
volatile int g_peak_set_current=FLASH_CURRENT_MA; // Peak flash current
volatile int measured_current;
volatile int16_t g_duty_cycle=DEF_DUTY_CYCLE;  // global to give debugger access
volatile int error;

volatile CTL_MODE_E control_mode=DEF_CONTROL_MODE;

uint8_t PREV_STATE = PRIORITY_STATE;		// my state choice... defaults to priority

int32_t pGain_8 = PGAIN_8; // proportional gain numerator scaled by 2^8
volatile int g_enable_flash=1;

SPid plantPID = {0, // dState
	0, // iState
	LIM_DUTY_CYCLE, // iMax
	-LIM_DUTY_CYCLE, // iMin
	P_GAIN_FL, // pGain
	I_GAIN_FL, // iGain
	D_GAIN_FL  // dGain
};

SPidFX plantPID_FX = {FL_TO_FX(0), // dState
	FL_TO_FX(0), // iState
	FL_TO_FX(LIM_DUTY_CYCLE), // iMax
	FL_TO_FX(-LIM_DUTY_CYCLE), // iMin
	P_GAIN_FX, // pGain
	I_GAIN_FX, // iGain
	D_GAIN_FX  // dGain
};

float UpdatePID(SPid * pid, float error, float position){
	float pTerm, dTerm, iTerm;

	// calculate the proportional term
	pTerm = pid->pGain * error;
	// calculate the integral state with appropriate limiting
	pid->iState += error;
	if (pid->iState > pid->iMax) 
		pid->iState = pid->iMax;
	else if (pid->iState < pid->iMin) 
		pid->iState = pid->iMin;
	iTerm = pid->iGain * pid->iState; // calculate the integral term
	dTerm = pid->dGain * (position - pid->dState);
	pid->dState = position;

	return pTerm + iTerm - dTerm;
}

FX16_16 UpdatePID_FX(SPidFX * pid, FX16_16 error_FX, FX16_16 position_FX){
	FX16_16 pTerm, dTerm, iTerm, diff, ret_val;

	// calculate the proportional term
	pTerm = Multiply_FX(pid->pGain, error_FX);

	// calculate the integral state with appropriate limiting
	pid->iState = Add_FX(pid->iState, error_FX);
	if (pid->iState > pid->iMax) 
		pid->iState = pid->iMax;
	else if (pid->iState < pid->iMin) 
		pid->iState = pid->iMin;
	
	iTerm = Multiply_FX(pid->iGain, pid->iState); // calculate the integral term
	diff = Subtract_FX(position_FX, pid->dState);
	dTerm = Multiply_FX(pid->dGain, diff);
	pid->dState = position_FX;

	ret_val = Add_FX(pTerm, iTerm);
	ret_val = Subtract_FX(ret_val, dTerm);
	return ret_val;
}

void Control_HBLED(void) {
	uint16_t res;
	FX16_16 change_FX, error_FX;
	
	FPTB->PSOR = MASK(DBG_CONTROLLER_POS);	// sets debug bit
	
#if USE_ADC_INTERRUPT
	// already completed conversion, so don't wait
#else
	while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK))
		; // wait until end of conversion
#endif
	res = ADC0->R[0];

	measured_current = (res*1500)>>16; // Extra Credit: Make this code work: V_REF_MV*MA_SCALING_FACTOR)/(ADC_FULL_SCALE*R_SENSE)

	switch (control_mode) {
		case OpenLoop:
				// don't do anything!
			break;
		case BangBang:
			if (measured_current < g_set_current)
				g_duty_cycle = LIM_DUTY_CYCLE;
			else
				g_duty_cycle = 0;
			break;
		case Incremental:
			if (measured_current < g_set_current)
				g_duty_cycle += INC_STEP;
			else
				g_duty_cycle -= INC_STEP;
			break;
		case Proportional:
			g_duty_cycle += (pGain_8*(g_set_current - measured_current))/256; //  - 1;
		break;
		case PID:
			g_duty_cycle += UpdatePID(&plantPID, g_set_current - measured_current, measured_current);
			break;
		case PID_FX:
			error_FX = INT_TO_FX(g_set_current - measured_current);
			change_FX = UpdatePID_FX(&plantPID_FX, error_FX, INT_TO_FX(measured_current));
			g_duty_cycle += FX_TO_INT(change_FX);
		break;
		default:
			break;
	}
	
	// Update PWM controller with duty cycle
	if (g_duty_cycle < 0)
		g_duty_cycle = 0;
	else if (g_duty_cycle > LIM_DUTY_CYCLE)
		g_duty_cycle = LIM_DUTY_CYCLE;
	PWM_Set_Value(TPM0, PWM_HBLED_CHANNEL, g_duty_cycle);
	FPTB->PCOR = MASK(DBG_CONTROLLER_POS);
}

// -------------------------------------------------------
#if USE_ADC_INTERRUPT
void ADC0_IRQHandler() {
	static ADC_SERVER_MSG_QUEUE message;
	static uint16_t result;
	
	FPTB->PSOR = MASK(DBG_IRQ_ADC_POS); // sets debug port
	
	if (PREV_STATE == PRIORITY_STATE) {
		Control_HBLED();
	}
	
	else if (PREV_STATE == QUEUE_STATE) {
		// get result
		result = ADC0->R[0];
		
		// store result
		pointer_to_result = &result;
		
		// signal thread with flag
		osThreadFlagsSet(message.TID, message.FlagNum);
	}
	else {
		Control_RGB_LEDs(1,0,0);
	}

	// ARE THERE ANY CONVERSION REQUESTS IN QUEUE?
	if (osMessageQueueGetCount(ADC_Server)) {
		// Set conversion type to queued
		PREV_STATE = QUEUE_STATE;
		
		// Get request from queue
		osMessageQueueGet(ADC_Server, &message, NULL, NULL);
		
		// configure ADC for software trigger
		// start conversion on channel
		restore_SW_trigger(message.ChannelNum);
		
	} else {
		// set conversion type to priority
		PREV_STATE = PRIORITY_STATE;
		
		// restore hw trigger
		// select priority channel for ADC
		restore_HW_trigger();
	}
	
	FPTB->PCOR = MASK(DBG_IRQ_ADC_POS); // clears debug port
} // END IRQ HANDLER
#endif

// -------------------------------------------------------
void restore_HW_trigger() {
	// Enable hardware triggering of ADC
	ADC0->SC2 |= ADC_SC2_ADTRG(1);
	// Select triggering by TPM0 Overflow
	SIM->SOPT7 = SIM_SOPT7_ADC0TRGSEL(8) | SIM_SOPT7_ADC0ALTTRGEN_MASK;
	// Select input channel 
	ADC0->SC1[0] &= ~ADC_SC1_ADCH_MASK;
	ADC0->SC1[0] |= ADC_SC1_ADCH(ADC_SENSE_CHANNEL);
} // end restore_HW_trigger

// -------------------------------------------------------
// @param channel - x or y channel that comes from message queue
void restore_SW_trigger(uint8_t channel) {
	// Enable software triggering of ADC
	ADC0->SC2 &= ~ADC_SC2_ADTRG(1);
	// Select input channel, starts the SW triggering
	ADC0->SC1[0] &= ~ADC_SC1_ADCH_MASK;
	ADC0->SC1[0] |= ADC_SC1_ADCH(channel);
	//ADC0->SC1[0] = channel;
	ADC0->SC1[0] |= ADC_SC1_AIEN(1);
} // end restore_SW_trigger
// -------------------------------------------------------

void Set_DAC(unsigned int code) {
	// Force 16-bit write to DAC
	uint16_t * dac0dat = (uint16_t *)&(DAC0->DAT[0].DATL);
	*dac0dat = (uint16_t) code;
}

void Set_DAC_mA(unsigned int current) {
	unsigned int code = MA_TO_DAC_CODE(current);
	// Force 16-bit write to DAC
	uint16_t * dac0dat = (uint16_t *)&(DAC0->DAT[0].DATL);
	*dac0dat = (uint16_t) code;
}

void Init_DAC_HBLED(void) {
  // Enable clock to DAC and Port E
	SIM->SCGC6 |= SIM_SCGC6_DAC0_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	// Select analog for pin
	PORTE->PCR[DAC_POS] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[DAC_POS] |= PORT_PCR_MUX(0);	
		
	// Disable buffer mode
	DAC0->C1 = 0;
	DAC0->C2 = 0;
	
	// Enable DAC, select VDDA as reference voltage
	DAC0->C0 = DAC_C0_DACEN_MASK | DAC_C0_DACRFS_MASK;
	Set_DAC(0);
}

void Init_ADC_HBLED(void) {
	// Configure ADC to read Ch 8 (FPTB 0)
	SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK; 
	ADC0->CFG1 = 0x0C; // 16 bit
	//	ADC0->CFG2 = ADC_CFG2_ADLSTS(3);
	ADC0->SC2 = ADC_SC2_REFSEL(0);

#if USE_ADC_HW_TRIGGER
	// Enable hardware triggering of ADC
	ADC0->SC2 |= ADC_SC2_ADTRG(1);
	// Select triggering by TPM0 Overflow
	SIM->SOPT7 = SIM_SOPT7_ADC0TRGSEL(8) | SIM_SOPT7_ADC0ALTTRGEN_MASK;
	// Select input channel 
	ADC0->SC1[0] &= ~ADC_SC1_ADCH_MASK;
	ADC0->SC1[0] |= ADC_SC1_ADCH(ADC_SENSE_CHANNEL);
#endif

#if USE_ADC_INTERRUPT 
	// enable ADC interrupt
	ADC0->SC1[0] |= ADC_SC1_AIEN(1);

	// Configure NVIC for ADC interrupt
	NVIC_SetPriority(ADC0_IRQn, 128); // 0, 64, 128 or 192
	NVIC_ClearPendingIRQ(ADC0_IRQn); 
	NVIC_EnableIRQ(ADC0_IRQn);	
#endif
}

void Update_Set_Current(void) {
	static int delay=FLASH_PERIOD;
	int current;
	
	if (g_enable_flash){
		delay--;
#if 1 // Original two-level flash code		
		if (delay == 40) {
			Set_DAC_mA(g_peak_set_current);
			g_set_current = g_peak_set_current;
		} else if ((delay < 40) & (delay > 0)) {
			delay=FLASH_PERIOD;
			Set_DAC_mA(g_peak_set_current/4);
			g_set_current = g_peak_set_current/4;
		} else {
			Set_DAC_mA(0);
			g_set_current = 0;
		}
#else // Flash brightness depends on roll angle
		if (delay==0) {
			current = (int) roll;
			if (current < 0)
				current = -current;
			Set_DAC_mA(current);
			g_set_current = current;
			delay=FLASH_PERIOD;
		} else {
			Set_DAC_mA(0);
			g_set_current = 0;
		}
#endif
	}
}

void Init_HBLED(void) {
	Init_DAC_HBLED();
	Init_ADC_HBLED();
	
	// Configure driver for buck converter
	// Set up PTE31 to use for SMPS with TPM0 Ch 4
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	PORTE->PCR[31]  &= PORT_PCR_MUX(7);
	PORTE->PCR[31]  |= PORT_PCR_MUX(3);
	PWM_Init(TPM0, PWM_HBLED_CHANNEL, PWM_PERIOD, 0, 0, 0);
	
}
