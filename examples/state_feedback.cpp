#include "state_feedback.hpp"
#include <iostream>

StateFeedbackController::StateFeedbackController(const ModelParameters& params,
                                                                                                 double phi0, double omega_phi0,
                                                                                                 double wn, double zeta_d)
        : m_(params.m), g_(params.g), l_cog_(params.l_cog),
            a_(params.a), b_(params.b), l_(params.l), Ix_(params.J),
            PWM_base_(params.PWM_base), zeta_(params.zeta),
            phi0_(phi0), omega_phi0_(omega_phi0),
            wn_(wn), zeta_d_(zeta_d),
            u0_(0.0), kg_(0.0), b2_(0.0), k1_(0.0), k2_(0.0)
{
        recomputeGains();
}

void StateFeedbackController::recomputeGains() {
    // denom coefficient used in multiple formulas: (4*a*PWM_base + 2*b)
    const double denomCoeff = 4.0 * a_ * PWM_base_ + 2.0 * b_;

    // u0 = (m*g*l_cog*sin(phi0)) / ((4 * a * PWM_base + 2 * b) * l * cos(zeta));
    double denomU0 = denomCoeff * l_ * std::cos(zeta_);
    if (std::abs(denomU0) < 1e-12) {
        std::cerr << "[StateFeedback] denom for u0 is zero, setting u0=0\n";
        u0_ = 0.0;
    } else {
        u0_ = (m_ * g_ * l_cog_ * std::sin(phi0_)) / denomU0;
    }

    // kg = (m*g*l_cog/Ix) * cos(phi0);
    if (std::abs(Ix_) < 1e-12) {
        std::cerr << "[StateFeedback] Ix is zero, kg set to 0\n";
        kg_ = 0.0;
    } else {
        kg_ = (m_ * g_ * l_cog_ / Ix_) * std::cos(phi0_);
    }

    // b2 = ((4*a*PWM_base + 2*b) * l * cos(zeta)) / Ix;
    if (std::abs(Ix_) < 1e-12) {
        std::cerr << "[StateFeedback] Ix is zero, b2 set to large value to avoid div0\n";
        b2_ = 1e12; // avoid division by zero downstream
    } else {
        b2_ = (denomCoeff * l_ * std::cos(zeta_)) / Ix_;
    }

    // k2 = (2*zeta_d*wn) / b2;
    if (std::abs(b2_) < 1e-12) {
        std::cerr << "[StateFeedback] b2 is zero, k2 set to 0\n";
        k2_ = 0.0;
    } else {
        k2_ = (2.0 * zeta_d_ * wn_) / b2_;
    }

    // k1 = (wn^2 + kg) / b2;
    if (std::abs(b2_) < 1e-12) {
        k1_ = 0.0;
    } else {
        k1_ = ((wn_ * wn_) + kg_) / b2_;
    }
}

double StateFeedbackController::compute(double phi, double omega_phi) const {
    // delta_x1 = x1 - x1_0 ; x1 is phi
    // delta_x2 = x2 - x2_0 ; x2 is omega_phi
    const double delta_x1 = phi - phi0_;
    const double delta_x2 = omega_phi - omega_phi0_;

    // u = u0 - k1*delta_x1 - k2*delta_x2
    return u0_ - k1_ * delta_x1 - k2_ * delta_x2;
}

void StateFeedbackController::setSetpoints(double phi0, double omega_phi0) {
    phi0_ = phi0;
    omega_phi0_ = omega_phi0;
    recomputeGains();
}

void StateFeedbackController::setDynamics(double wn, double zeta_d) {
    wn_ = wn;
    zeta_d_ = zeta_d;
    recomputeGains();
}
