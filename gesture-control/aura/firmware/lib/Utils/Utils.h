#ifndef _UTILS_H
#define _UTILS_H

#include "math.h"
#include <Arduino.h>

int alphaFilter(int currentValue, int previousValue, double alpha);

int inputExponential(int expo, long int value, int inputMin, int inputMax);

#endif // _UTILS_H
