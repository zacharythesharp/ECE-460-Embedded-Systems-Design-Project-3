#include <MKL25Z4.H>
#include "cmsis_os2.h"

#define PRIORITY_STATE 	(1)
#define QUEUE_STATE			(2)

typedef struct {
	uint8_t ChannelNum;
	uint16_t *ResultPtr;
	osThreadId_t TID;
	uint8_t FlagNum;
} ADC_SERVER_MSG_QUEUE;

void restore_HW_trigger(void);
void restore_SW_trigger(uint8_t channel);
