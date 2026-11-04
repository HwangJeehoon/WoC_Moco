#include "EffortPropulsionIntegrator.h"

#include <OpenSim/Simulation/Model/Model.h>

#include <cmath>

using namespace OpenSim;

EffortPropulsionIntegrator::EffortPropulsionIntegrator() {
    constructProperties();
}

void EffortPropulsionIntegrator::constructProperties() {
    constructProperty_gate_sigma(1.0);
    constructProperty_gate_threshold(5.0);
    constructProperty_impulse_scale(1.0);
    constructProperty_include_mean_force(false);
    constructProperty_include_metabolic_energy(false);
}

void EffortPropulsionIntegrator::extendFinalizeFromProperties() {
    Super::extendFinalizeFromProperties();
    OPENSIM_THROW_IF_FRMOBJ(
            !std::isfinite(get_gate_sigma()) || get_gate_sigma() <= 0,
            Exception, "gate_sigma must be a positive finite value.");
    OPENSIM_THROW_IF_FRMOBJ(!std::isfinite(get_gate_threshold()), Exception,
            "gate_threshold must be finite.");
    OPENSIM_THROW_IF_FRMOBJ(!std::isfinite(get_impulse_scale()) ||
                                    get_impulse_scale() <= 0,
            Exception, "impulse_scale must be a positive finite value.");
    for (const auto* input : {"right_contact_forces", "left_contact_forces"}) {
        OPENSIM_THROW_IF_FRMOBJ(getInput(input).getNumConnectees() == 0,
                Exception, fmt::format("Input '{}' has no connectees.", input));
    }
    OPENSIM_THROW_IF_FRMOBJ(get_include_metabolic_energy() &&
                                    getInput("metabolic_rate")
                                            .getConnecteePath()
                                            .empty(),
            Exception,
            "include_metabolic_energy requires the input 'metabolic_rate' to "
            "be connected.");
}

void EffortPropulsionIntegrator::extendAddToSystem(
        SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);
    addStateVariable("effort_integral");
    addStateVariable("propulsive_impulse");
    if (get_include_mean_force()) {
        addStateVariable("mean_propulsive_force");
        addStateVariable("mean_force_residual");
    }
    if (get_include_metabolic_energy()) {
        addStateVariable("metabolic_energy");
    }
}

void EffortPropulsionIntegrator::computeStateVariableDerivatives(
        const SimTK::State& s) const {
    // The base class has no states of its own (calling Super would throw).
    setStateVariableDerivativeValue(s, "effort_integral", calcEffortRate(s));
    setStateVariableDerivativeValue(
            s, "propulsive_impulse", calcPropulsiveImpulseRate(s));
    if (get_include_mean_force()) {
        setStateVariableDerivativeValue(s, "mean_propulsive_force", 0.0);
        setStateVariableDerivativeValue(
                s, "mean_force_residual", calcMeanForceResidualRate(s));
    }
    if (get_include_metabolic_energy()) {
        // The metabolic rate output is computed at Stage::Dynamics; state
        // derivatives are evaluated while realizing Stage::Acceleration.
        setStateVariableDerivativeValue(s, "metabolic_energy",
                getInput<double>("metabolic_rate").getValue(s));
    }
}

double EffortPropulsionIntegrator::calcEffortRate(const SimTK::State& s) const {
    double rate = 0;
    for (const auto& actuator : getModel().getComponentList<Actuator>()) {
        if (!actuator.get_appliesForce()) { continue; }
        // Assistance is not part of the biological effort budget.
        const auto& name = actuator.getName();
        if (name == "AFO_r" || name == "AFO_l") { continue; }
        const auto controls = actuator.getControls(s);
        for (int i = 0; i < controls.size(); ++i) {
            rate += controls[i] * controls[i];
        }
    }
    return rate;
}

double EffortPropulsionIntegrator::calcFootForce(
        const SimTK::State& s, const std::string& input) const {
    const auto& forces = getInput<SimTK::SpatialVec>(input);
    double force = 0;
    for (unsigned i = 0; i < forces.getNumConnectees(); ++i) {
        // SpatialVec = [torque; force]: [1][0] is force-x (flat index 3).
        force += forces.getValue(s, i)[1][0];
    }
    return force;
}

double EffortPropulsionIntegrator::calcGate(double force) const {
    // Logistic function evaluated so that exp() never overflows: exp() only
    // receives a non-positive argument (same as MeanPropulsiveForce).
    const double x = (force - get_gate_threshold()) / get_gate_sigma();
    if (x >= 0) { return 1.0 / (1.0 + std::exp(-x)); }
    const double e = std::exp(x);
    return e / (1.0 + e);
}

double EffortPropulsionIntegrator::calcPropulsiveImpulseRate(
        const SimTK::State& s) const {
    const double right = calcFootForce(s, "right_contact_forces");
    const double left = calcFootForce(s, "left_contact_forces");
    return right * calcGate(right) + left * calcGate(left);
}

double EffortPropulsionIntegrator::calcPropulsiveTimeRate(
        const SimTK::State& s) const {
    return calcGate(calcFootForce(s, "right_contact_forces")) +
           calcGate(calcFootForce(s, "left_contact_forces"));
}

double EffortPropulsionIntegrator::calcMeanForceResidualRate(
        const SimTK::State& s) const {
    const double m = getStateVariableValue(s, "mean_propulsive_force");
    const double right = calcFootForce(s, "right_contact_forces");
    const double left = calcFootForce(s, "left_contact_forces");
    return calcGate(right) * (right - m) + calcGate(left) * (left - m);
}

double EffortPropulsionIntegrator::getEffortIntegral(
        const SimTK::State& s) const {
    return getStateVariableValue(s, "effort_integral");
}

double EffortPropulsionIntegrator::getScaledPropulsiveImpulse(
        const SimTK::State& s) const {
    return get_impulse_scale() * getStateVariableValue(s, "propulsive_impulse");
}

double EffortPropulsionIntegrator::getMetabolicEnergy(
        const SimTK::State& s) const {
    return getStateVariableValue(s, "metabolic_energy");
}
