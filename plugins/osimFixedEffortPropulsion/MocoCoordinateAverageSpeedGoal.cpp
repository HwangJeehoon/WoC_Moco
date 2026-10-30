#include "MocoCoordinateAverageSpeedGoal.h"

#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;

void MocoCoordinateAverageSpeedGoal::constructProperties() {
    constructProperty_coordinate_path("");
}

void MocoCoordinateAverageSpeedGoal::initializeOnModelImpl(
        const Model& model) const {
    m_coordinate.reset(&model.getComponent<Coordinate>(get_coordinate_path()));
    // Only the coordinate values (Q) of the end states are read.
    setRequirements(0, 1, SimTK::Stage::Position);
}

void MocoCoordinateAverageSpeedGoal::calcGoalImpl(
        const GoalInput& input, SimTK::Vector& goal) const {
    const double displacement = m_coordinate->getValue(input.final_state) -
                                m_coordinate->getValue(input.initial_state);
    goal[0] = displacement / (input.final_time - input.initial_time);
}

void MocoCoordinateAverageSpeedGoal::printDescriptionImpl() const {
    log_info("        coordinate: {}", get_coordinate_path());
}
