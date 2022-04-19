#include "utils.h"

double sat(double val, double min, double max) {
    return val < min ? min : (val > max ? max : val);
}