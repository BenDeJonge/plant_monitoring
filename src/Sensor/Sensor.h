#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>

bool toggleForTime(int pin, unsigned long startTime, int duration, int defaultState);
float mapSensor(int pin, int voltage);
float linRegSensor(float reading, float slope, float intercept);

#endif