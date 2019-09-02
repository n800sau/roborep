#include <Arduino.h>

const double COUNT_PER_REV = 625.0; // truck wheel 18 stripes (625 edges per rev)
const double WHEEL_DIAMETER = 0.05; // truck wheels
// distance between wheels
const double WHEEL_TRACK = 0.082; // truck width
const double ENC_STEP = WHEEL_DIAMETER * PI / COUNT_PER_REV;
const double TICKS_PER_METER = 1 / ENC_STEP;
//const double GEAR_REDUCTION = 50 / 12. * 50 / 12.; // from doc for chassis ???

