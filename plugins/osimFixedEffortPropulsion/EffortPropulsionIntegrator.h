#ifndef OSIM_EFFORT_PROPULSION_INTEGRATOR_H
#define OSIM_EFFORT_PROPULSION_INTEGRATOR_H

#include "osimFixedEffortPropulsionDLL.h"

#include <OpenSim/Simulation/Model/ModelComponent.h>

namespace OpenSim {

/** Integrates the effort and the smooth positive AP impulse over a phase so
that both can be constrained at the final time (used by moco_WoC_loop for
opts.mocoObjectiveMode = 'minSpeedFixedEP').

    effort_rate           = sum_c u_c^2
        c: every control of every actuator with appliesForce = true, weight 1.
        This is the integrand of MocoControlGoal with its defaults (exponent
        2, no control weights, ignore_controlled_actuators = false), so
        controls set by a PrescribedController are included.
    positive_impulse_rate = softplus_s(F_right) + softplus_s(F_left)   [N]
        F_side = sum of the force-x components of the side's contact-force
        outputs (SpatialVec = [torque; force], force-x is [1][0]); each
        foot's forces are summed before the positive part.
        softplus_s(F) = log(1 + exp(s*F)) / s, the smooth maximum of
        MocoOutputExtremumGoal ('maximum', exponent 1), s = smoothing_factor.

States (bound both to 0 at the initial time in the MocoProblem):
    effort_integral  = integral of effort_rate
    positive_impulse = integral of positive_impulse_rate   [N*s]

Outputs effort_integral and scaled_positive_impulse (= impulse_scale *
positive_impulse) are meant for MocoFinalOutputGoal endpoint constraints:
effort_integral with divide_by_displacement gives the MocoControlGoal value
(1/d) * integral of sum_c u_c^2 dt. impulse_scale converts the solved phase
to the target phase (2 for a half stride solved with left-right symmetry).

Integrating through states keeps the NLP sparse: an integral endpoint
constraint MocoGoal has a constraint Jacobian row over every grid point,
which made each Jacobian evaluation about 27 times slower in Moco 4.5.2. */
class OSIMFIXEDEFFORTPROPULSION_API EffortPropulsionIntegrator
        : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT(EffortPropulsionIntegrator, ModelComponent);

public:
    OpenSim_DECLARE_PROPERTY(smoothing_factor, double,
            "Smoothing factor s of softplus_s. Default: 0.2.");
    OpenSim_DECLARE_PROPERTY(impulse_scale, double,
            "Factor of scaled_positive_impulse. Default: 1.");

    OpenSim_DECLARE_LIST_INPUT(right_contact_forces, SimTK::SpatialVec,
            SimTK::Stage::Dynamics,
            "Right-foot contact forces, e.g. /contactHeel_r|sphere_force.");
    OpenSim_DECLARE_LIST_INPUT(left_contact_forces, SimTK::SpatialVec,
            SimTK::Stage::Dynamics,
            "Left-foot contact forces, e.g. /contactHeel_l|sphere_force.");

    OpenSim_DECLARE_OUTPUT(effort_rate, double, calcEffortRate,
            SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(positive_impulse_rate, double,
            calcPositiveImpulseRate, SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(effort_integral, double, getEffortIntegral,
            SimTK::Stage::Model);
    OpenSim_DECLARE_OUTPUT(scaled_positive_impulse, double,
            getScaledPositiveImpulse, SimTK::Stage::Model);

    EffortPropulsionIntegrator();

    double calcEffortRate(const SimTK::State& s) const;
    double calcPositiveImpulseRate(const SimTK::State& s) const;
    double getEffortIntegral(const SimTK::State& s) const;
    double getScaledPositiveImpulse(const SimTK::State& s) const;

protected:
    void extendFinalizeFromProperties() override;
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    void computeStateVariableDerivatives(const SimTK::State& s) const override;

private:
    void constructProperties();
    double calcFootForce(const SimTK::State& s, const std::string& input) const;
};

} // namespace OpenSim

#endif // OSIM_EFFORT_PROPULSION_INTEGRATOR_H
