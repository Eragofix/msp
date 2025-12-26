#pragma once

#include <cmath>

struct ModelParameters {
    double a = 1.766e-6;
    double b = 0.001346;
    double l = 0.11;
    double zeta = M_PI / 4.0;
    double J = 0.009; // Ix
    double m = 0.261;
    double g = 9.81;
    double l_cog = 0.03;
    double PWM_base = 1200.0;
};
