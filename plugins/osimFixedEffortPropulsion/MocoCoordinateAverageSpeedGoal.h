#ifndef OSIM_MOCO_COORDINATE_AVERAGE_SPEED_GOAL_H
#define OSIM_MOCO_COORDINATE_AVERAGE_SPEED_GOAL_H

#include "osimFixedEffortPropulsionDLL.h"

#include <OpenSim/Moco/MocoGoal/MocoGoal.h>

namespace OpenSim {

class Coordinate;

/** Average speed of a coordinate over the phase:

    value = (q(t_final) - q(t_initial)) / (t_final - t_initial)

Unlike MocoAverageSpeedGoal (center of mass, matched to a desired speed),
the value itself is the cost; with a positive weight Moco minimizes the
average speed. */
class OSIMFIXEDEFFORTPROPULSION_API MocoCoordinateAverageSpeedGoal
        : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoCoordinateAverageSpeedGoal, MocoGoal);

public:
    OpenSim_DECLARE_PROPERTY(coordinate_path, std::string,
            "Path to the coordinate, e.g. /jointset/groundPelvis/pelvis_tx.");

    MocoCoordinateAverageSpeedGoal() { constructProperties(); }
    MocoCoordinateAverageSpeedGoal(std::string name)
            : MocoGoal(std::move(name)) {
        constructProperties();
    }
    MocoCoordinateAverageSpeedGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {
        constructProperties();
    }

protected:
    void initializeOnModelImpl(const Model&) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& goal) const override;
    void printDescriptionImpl() const override;

private:
    void constructProperties();

    mutable SimTK::ReferencePtr<const Coordinate> m_coordinate;
};

} // namespace OpenSim

#endif // OSIM_MOCO_COORDINATE_AVERAGE_SPEED_GOAL_H
