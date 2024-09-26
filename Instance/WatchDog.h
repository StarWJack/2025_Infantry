#ifndef WATCHDOG_H
#define WATCHDOG_H

#include "main.h"
#include "config.h"

typedef struct{
	uint32_t last_time;
	uint8_t status;
} WatchDog_t;
extern WatchDog_t remote_WatchDog;
extern WatchDog_t referee_WatchDog;
extern WatchDog_t motor_WatchDog[MOTOR_NUM];

void feedDog(WatchDog_t* hdog);
uint8_t Dog_Status_update(WatchDog_t* hdog);
void WatchDogInit(void);
void WatchDogTask(void);

#endif
