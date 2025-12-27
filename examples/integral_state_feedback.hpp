#pragma once

#include <cmath>
#include <iostream>
#include <algorithm>
#include "model_parameters.hpp"

class IntegralStateFeedbackController {
public:
    IntegralStateFeedbackController(const ModelParameters& params,
                                    double phi0, double omega_phi0,
                                    double wn, double zeta_d, double pI)
        : m_(params.m), g_(params.g), l_cog_(params.l_cog),
          a_(params.a), b_(params.b), l_(params.l), Ix_(params.J),
          PWM_base_(params.PWM_base), zeta_(params.zeta),
          phi0_(phi0), omega_phi0_(omega_phi0),
          wn_(wn), zeta_d_(zeta_d), pI_(pI),
          u0_(0.0), kg_(0.0), b2_(0.0), k1_(0.0), k2_(0.0), Fi_(0.0), xi_(0.0)
    {
        recomputeGains();
    }

    // compute control with timestep dt (seconds)
    double compute(double phi, double omega_phi, double dt) {
        const double delta_x1 = phi - phi0_;
        const double delta_x2 = omega_phi - omega_phi0_;

        if (!(dt > 0.0)) dt = 0.0;

        xi_ += delta_x1 * dt;

        double u = u0_ - k1_ * delta_x1 - k2_ * delta_x2 - Fi_ * xi_;
        return u;
    }

    void setSetpoints(double phi0, double omega_phi0) {
        phi0_ = phi0;
        omega_phi0_ = omega_phi0;
        recomputeGains();
    }

    void setDynamics(double wn, double zeta_d, double pI) {
        wn_ = wn;
        zeta_d_ = zeta_d;
        pI_ = pI;
        recomputeGains();
    }

    double getU0() const { return u0_; }
    double getK1() const { return k1_; }
    double getK2() const { return k2_; }
    double getFi() const { return Fi_; }
    double getXi() const { return xi_; }

    void resetIntegrator() { xi_ = 0.0; }

private:
    void recomputeGains() {
        const double denomCoeff = 4.0 * a_ * PWM_base_ + 2.0 * b_;

        double denomU0 = denomCoeff * l_ * std::cos(zeta_);
        if (std::abs(denomU0) < 1e-12) {
            std::cerr << "[IntegralStateFeedback] denom for u0 is zero, setting u0=0\n";
            u0_ = 0.0;
        } else {
            u0_ = -(m_ * g_ * l_cog_ * std::sin(phi0_)) / denomU0;
        }

        if (std::abs(Ix_) < 1e-12) {
            std::cerr << "[IntegralStateFeedback] Ix is zero, kg set to 0 and b2 large\n";
            kg_ = 0.0;
            b2_ = 1e12;
        } else {
            kg_ = (m_ * g_ * l_cog_ / Ix_) * std::cos(phi0_);
            b2_ = (denomCoeff * l_ * std::cos(zeta_)) / Ix_;
        }

        const double a2 = 2.0 * zeta_d_ * wn_ + pI_;
        const double a1 = (wn_ * wn_) + 2.0 * zeta_d_ * wn_ * pI_;
        const double a0 = (wn_ * wn_) * pI_;

        if (std::abs(b2_) < 1e-12) {
            std::cerr << "[IntegralStateFeedback] b2 is zero, gains set to 0\n";
            k2_ = 0.0;
            k1_ = 0.0;
            Fi_ = 0.0;
        } else {
            k2_ = a2 / b2_;
            k1_ = (a1 + kg_) / b2_;
            Fi_ = a0 / b2_;
        }
    }

    // model params
    double m_, g_, l_cog_;
    double a_, b_, l_, Ix_;
    double PWM_base_, zeta_;

    // setpoints / dynamics
    double phi0_, omega_phi0_;
    double wn_, zeta_d_, pI_;

    // precomputed terms / states
    double u0_;
    double kg_;
    double b2_;
    double k1_;
    double k2_;
    double Fi_;
    double xi_; // integrator state
};
