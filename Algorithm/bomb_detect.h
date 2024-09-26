#ifndef BOMB_DETECT_H
#define BOMB_DETECT_H

#include "main.h"
#include "config.h"

typedef struct{
	uint8_t status;
} Bomb_StatusTypedef;

extern Bomb_StatusTypedef bomb_status;

#endif