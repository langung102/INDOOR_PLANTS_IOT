#ifndef ULTRASONIC
#define ULTRASONIC

#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701

#include "ultrasonic.h"
#include "global.h"
#include "Arduino.h"

int get_distance_cm();
int get_distance_inch();

#endif
