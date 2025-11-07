#include "filter.h"

float Filter::filterMedian3Vals(float *newValues) {

    std::sort(newValues, newValues + 3);
    return newValues[1];
}

float Filter::filterLowPass(float medianValue) {

    savedValue = savedValue * alpha + medianValue * (1 - alpha);
    return savedValue;
}

float Filter::doFiltering3Vals(float *newValues) {

    float medianValue = filterMedian3Vals(newValues);
    float lowPassedValue = filterLowPass(medianValue);
    return lowPassedValue;
}
