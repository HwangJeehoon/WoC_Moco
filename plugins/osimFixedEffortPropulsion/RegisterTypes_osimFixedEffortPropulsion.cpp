#include "EffortPropulsionIntegrator.h"
#include "MocoCoordinateAverageSpeedGoal.h"

// Register the plugin types when the library is loaded (e.g., with
// opensimCommon.LoadOpenSimLibraryExact), so that OpenSim can create them by
// name and read them from XML.
namespace {
struct Registrar {
    Registrar() {
        OpenSim::Object::registerType(OpenSim::EffortPropulsionIntegrator());
        OpenSim::Object::registerType(OpenSim::MocoCoordinateAverageSpeedGoal());
    }
} registrar;
} // namespace
