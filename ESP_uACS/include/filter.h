#ifndef FILTER_H
#define FILTER_H

#include <algorithm>

class Filter {

    float alpha;
    float savedValue = 0;

    float filterMedian3Vals(float *newValues);
    float filterLowPass(float medianValue);

public:

    Filter(float alpha) {
        this->alpha = alpha;
    }

    float doFiltering3Vals(float *newValues);
};

#endif
