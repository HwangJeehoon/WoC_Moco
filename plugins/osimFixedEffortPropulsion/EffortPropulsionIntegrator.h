#ifndef OSIM_EFFORT_PROPULSION_INTEGRATOR_H
#define OSIM_EFFORT_PROPULSION_INTEGRATOR_H

#include "osimFixedEffortPropulsionDLL.h"

#include <OpenSim/Simulation/Model/ModelComponent.h>

namespace OpenSim {

/** Integrates the effort and the gated propulsive impulse over a phase so
that both can be constrained at the final time (used by moco_WoC_loop for
opts.mocoObjectiveMode = 'minSpeedFixedEP').

    effort_rate = sum_c u_c^2
        c: every control of every actuator with appliesForce = true, weight 1.
        This is the integrand of MocoControlGoal with its defaults (exponent
        2, no control weights, ignore_controlled_actuators = false), so
        controls set by a PrescribedController are included.
    propulsive_impulse_rate = F_r g(F_r) + F_l g(F_l)                  [N]
    propulsive_time_rate    = g(F_r) + g(F_l)                          [-]
        F_side = sum of the force-x components of the side's contact-force
        outputs (SpatialVec = [torque; force], force-x is [1][0]); each foot
        is gated separately and then summed.
        g(F) = 1 / (1 + exp(-(F - F0) / sigma)), F0 = gate_threshold,
        sigma = gate_sigma.
    These are the rates of MeanPropulsiveForce (meanPropPerVel_goal branch):
    integrated over a stride they give Iplus [N*s] and Tplus [s], and
    Iplus / Tplus is its mean propulsive force.

States (bound both to 0 at the initial time in the MocoProblem):
    effort_integral     = integral of effort_rate
    propulsive_impulse  = integral of propulsive_impulse_rate   [N*s]

Outputs effort_integral and scaled_propulsive_impulse (= impulse_scale *
propulsive_impulse) are meant for MocoFinalOutputGoal endpoint constraints:
effort_integral with divide_by_displacement gives the MocoControlGoal value
(1/d) * integral of sum_c u_c^2 dt. impulse_scale converts the solved phase
to the target phase (2 for a half stride solved with left-right symmetry).
Both outputs depend on Stage::Time: MocoCasADiSolver copies the optimizer's
state values into the SimTK::State only for goals whose stage dependency is
at least Stage::Time, so a lower stage would read stale states.

Integrating through states keeps the NLP sparse: an integral endpoint
constraint MocoGoal has a constraint Jacobian row over every grid point,
which made each Jacobian evaluation about 27 times slower in Moco 4.5.2. */
class OSIMFIXEDEFFORTPROPULSION_API EffortPropulsionIntegrator
        : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT(EffortPropulsionIntegrator, ModelComponent);

public:
    OpenSim_DECLARE_PROPERTY(gate_sigma, double,
            "Gate smoothing width sigma (N). Must be positive. Default: 1.");
    OpenSim_DECLARE_PROPERTY(gate_threshold, double,
            "Gate center F0 (N); g(F0) = 0.5. Default: 5.");
    OpenSim_DECLARE_PROPERTY(impulse_scale, double,
            "Factor of scaled_propulsive_impulse. Default: 1.");

    OpenSim_DECLARE_LIST_INPUT(right_contact_forces, SimTK::SpatialVec,
            SimTK::Stage::Dynamics,
            "Right-foot contact forces, e.g. /contactHeel_r|sphere_force.");
    OpenSim_DECLARE_LIST_INPUT(left_contact_forces, SimTK::SpatialVec,
            SimTK::Stage::Dynamics,
            "Left-foot contact forces, e.g. /contactHeel_l|sphere_force.");

    OpenSim_DECLARE_OUTPUT(effort_rate, double, calcEffortRate,
            SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(propulsive_impulse_rate, double,
            calcPropulsiveImpulseRate, SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(propulsive_time_rate, double,
            calcPropulsiveTimeRate, SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(effort_integral, double, getEffortIntegral,
            SimTK::Stage::Time);
    OpenSim_DECLARE_OUTPUT(scaled_propulsive_impulse, double,
            getScaledPropulsiveImpulse, SimTK::Stage::Time);

    EffortPropulsionIntegrator();

    double calcEffortRate(const SimTK::State& s) const;
    double calcPropulsiveImpulseRate(const SimTK::State& s) const;
    double calcPropulsiveTimeRate(const SimTK::State& s) const;
    double getEffortIntegral(const SimTK::State& s) const;
    double getScaledPropulsiveImpulse(const SimTK::State& s) const;

protected:
    void extendFinalizeFromProperties() override;
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    void computeStateVariableDerivatives(const SimTK::State& s) const override;

private:
    void constructProperties();
    double calcFootForce(const SimTK::State& s, const std::string& input) const;
    double calcGate(double force) const;
};

} // namespace OpenSim

#endif // OSIM_EFFORT_PROPULSION_INTEGRATOR_H
