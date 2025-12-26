#pragma once

#include <cmath>
#include "model_parameters.hpp"

class StateFeedbackController {
public:
    // Constructor: provide model parameters struct, setpoints and controller dynamics
    StateFeedbackController(const ModelParameters& params,
                            double phi0, double omega_phi0,
                            double wn, double zeta_d);

    // Compute control output u given current phi and omega_phi
    double compute(double phi, double omega_phi) const;

    // Update setpoints
    void setSetpoints(double phi0, double omega_phi0);
    // Update desired closed-loop dynamics
    void setDynamics(double wn, double zeta_d);

    // Accessors
    double getU0() const { return u0_; }
    double getK1() const { return k1_; }
    double getK2() const { return k2_; }

private:
    void recomputeGains();

    double m_, g_, l_cog_;
    double a_, b_, l_, Ix_;
    double PWM_base_, zeta_;

    double phi0_, omega_phi0_;
    double wn_, zeta_d_;

    // precomputed terms
    double u0_;
    double kg_;
    double b2_;
    double k1_;
    double k2_;
};