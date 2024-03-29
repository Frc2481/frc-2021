#include "Utils/Interpolate.h"
#include "Utils/NormalizeToRange.h"

double interpolate::interp(
    const std::vector<double> &xData,
    const std::vector<double> &yData,
    double x,
    bool extrapolate) {

    // find index of interval for interpolation
    int i = 0;
    if(x >= xData[xData.size() - 2]) { // check if beyond right limit
        i = xData.size() - 2;
    }
    else {
        while(x > xData[i + 1]) {
            i++;
        }
    }

    // get interval for interpolation
    double xL = xData[i];
    double yL = yData[i];
    double xR = xData[i + 1];
    double yR = yData[i + 1];

    // do not extrapolate if set by setting y values to be equal
    if(!extrapolate) {
        // check if left limit
        if(x < xL) {
            yR = yL;
        }
        // check if right limit
        if(x > xR) {
            yL = yR;
        }
    }

    // calculate gradient
    double dydx = (yR - yL) / (xR - xL);

    // return linearly interpolated value
    return yL + dydx * (x - xL);
}

double interpolate::rangedInterp(
    const std::vector<double> &xData,
    const std::vector<double> &yData,
    double x,
    bool extrapolate,
    double rangeMin,
    double rangeMax) {

    // find index of interval for interpolation
    int i = 0;
    if(x >= xData[xData.size() - 2]) { // check if beyond right limit
        i = xData.size() - 2;
    }
    else {
        while(x > xData[i + 1]) {
            i++;
        }
    }

    // get interval for interpolation
    double xL = xData[i];
    double yL = yData[i];
    double xR = xData[i + 1];
    double yR = yData[i + 1];

    // do not extrapolate by setting y values to be equal
    if(!extrapolate) {
        // check if left limit
        if(x < xL) {
            yR = yL;
        }
        // check if right limit
        if(x > xR) {
            yL = yR;
        }
    }

    // calculate gradient
    double dydx = normalizeToRange::RangedDifference(yR - yL, rangeMin, rangeMax) / (xR - xL);

    // return linearly interpolated value
    return normalizeToRange::NormalizeToRange(yL + dydx * (x - xL), -180, 180, true);
}
