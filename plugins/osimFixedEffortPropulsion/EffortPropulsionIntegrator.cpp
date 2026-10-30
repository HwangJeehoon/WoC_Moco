#include "EffortPropulsionIntegrator.h"

#include <OpenSim/Simulation/Model/Model.h>

#include <cmath>

using namespace OpenSim;

EffortPropulsionIntegrator::EffortPropulsionIntegrator() {
    constructProperties();
}

void EffortPropulsionIntegrator::constructProperties() {
    constructProperty_smoothing_factor(0.2);
    constructProperty_impulse_scale(1.0);
}

void EffortPropulsionIntegrator::extendFinalizeFromProperties() {
    Super::extendFinalizeFromProperties();
    OPENSIM_THROW_IF_FRMOBJ(!std::isfinite(get_smoothing_factor()) ||
                                    get_smoothing_factor() <= 0,
            Exception, "smoothing_factor must be a positive finite value.");
    OPENSIM_THROW_IF_FRMOBJ(!std::isfinite(get_impulse_scale()) ||
                                    get_impulse_scale() <= 0,
            Exception, "impulse_scale must be a positive finite value.");
    for (const auto* input : {"right_contact_forces", "left_contact_forces"}) {
        OPENSIM_THROW_IF_FRMOBJ(getInput(input).getNumConnectees() == 0,
                Exception, fmt::format("Input '{}' has no connectees.", input));
    }
}

void EffortPropulsionIntegrator::extendAddToSystem(
        SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);
    addStateVariable("effort_integral");
    addStateVariable("positive_impulse");
}

void EffortPropulsionIntegrator::computeStateVariableDerivatives(
        const SimTK::State& s) const {
    // The base class has no states of its own (calling Super would throw).
    setStateVariableDerivativeValue(s, "effort_integral", calcEffortRate(s));
    setStateVariableDerivativeValue(
            s, "positive_impulse", calcPositiveImpulseRate(s));
}

double EffortPropulsionIntegrator::calcEffortRate(const SimTK::State& s) const {
    double rate = 0;
    for (const auto& actuator : getModel().getComponentList<Actuator>()) {
        if (!actuator.get_appliesForce()) { continue; }
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

double EffortPropulsionIntegrator::calcPositiveImpulseRate(
        const SimTK::State& s) const {
    // log(1 + exp(x)) written so that exp() cannot overflow.
    const double sf = get_smoothing_factor();
    double rate = 0;
    for (const auto* input : {"right_contact_forces", "left_contact_forces"}) {
        const double x = sf * calcFootForce(s, input);
        rate += (std::max(x, 0.0) + std::log1p(std::exp(-std::abs(x)))) / sf;
    }
    return rate;
}

double EffortPropulsionIntegrator::getEffortIntegral(
        const SimTK::State& s) const {
    return getStateVariableValue(s, "effort_integral");
}

double EffortPropulsionIntegrator::getScaledPositiveImpulse(
        const SimTK::State& s) const {
    return get_impulse_scale() * getStateVariableValue(s, "positive_impulse");
}
