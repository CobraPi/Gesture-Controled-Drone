#include "MathUtils.h"

int alphaFilter(int currentValue, int previousValue, double alpha) {
    return ((1 - alpha) * previousValue) + (alpha * currentValue);
}

int inputExponential(int expo, long int value, int inputMin, int inputMax) {
    // Return the exact input value for a linear response
    if (expo == 0) {
        return value;
    }

    // Calculate exponent magnitude
    double exponent = 3 * (expo / 100.00);

    // Store endpoints of exponential range
    long int min = pow(-100, 3);
    long int max = pow(100, 3);

    // Map the input value into the exponential range
    value = map(value,
                inputMin,
                inputMax,
                -100,
                100);

    // Apply the exponential function to the input value
    value = pow(value, exponent);

    // Return the modified value and map it back into it's original range
    return map(value,
               min,
               max,
               inputMin,
               inputMax);
}
