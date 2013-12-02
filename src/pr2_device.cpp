#include "sot_pr2/pr2_device.h"

#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/all-commands.h>

namespace sot_pr2 {

const double Pr2Device::TIMESTEP_DEFAULT = 0.001;

Pr2Device::Pr2Device(const std::string &name)
: dynamicgraph::sot::Device(name),
  timestep_(TIMESTEP_DEFAULT),
  previous_state_(),
  robotState_ ("StackOfTasks(" + name + ")::output(vector)::robotState"),
  pose(),
  baseff_(),
  loop_count_(0)
{
    sotDEBUGIN(25);
    signalRegistration(robotState_);
    baseff_.resize(12);

    std::string docstring;
    docstring =
       "\n"
       "    Integrate dynamics for time step provided as input\n"
       "\n"
       "      take one floating point number as input\n"
       "\n";
     addCommand("increment",
            dynamicgraph::command::makeCommandVoid1((Device&)*this,
                     &Device::increment, docstring));

     sotDEBUGOUT(25);
}

Pr2Device::~Pr2Device() {
}

void
Pr2Device::setSensors(SensorMap &sensorsIn) {
    sotDEBUGIN(25);
    SensorMap::iterator it;
    int t = stateSOUT.getTime() + 1;

    // Joints
    it = sensorsIn.find("joints");
    if (it != sensorsIn.end()) {
        const std::vector<double> &anglesIn = it->second.getValues();
        mlRobotState.resize(anglesIn.size() + 6);
        for (unsigned i=0; i<6; ++i)
            mlRobotState(i) = 0.;
        updateRobotState(anglesIn);
    }

    sotDEBUGOUT(25);
}

void
Pr2Device::setupSetSensors(SensorMap &sensorsIn) {
    setSensors(sensorsIn);
}

void
Pr2Device::nominalSetSensors(SensorMap &sensorsIn) {
    setSensors(sensorsIn);
}

void
Pr2Device::cleanupSetSensors(SensorMap &sensorsIn) {
    setSensors(sensorsIn);
}

void
Pr2Device::getControl(ControlMap &controlOut) {
    sotDEBUGIN(25);
    std::vector<double> anglesOut;
    anglesOut.resize(state_.size());

    try { increment(timestep_); }
    catch (...) {
        //std::cout << "Increment error (" << loop_count_ << ") (" << controlSIN << ")" << std::endl;
    }
     //++loop_count_;

    sotDEBUG(25) << "state = " << state_ << std::endl;
    sotDEBUG(25) << "diff = " << ((previous_state_.size() == state_.size()) ?
                                      (state_ - previous_state_) : state_ ) << std::endl;
    previous_state_ = state_;

    // Specify joint values
    if (anglesOut.size() != state_.size() - 6)
        anglesOut.resize(state_.size() - 6);
    for (unsigned int i=6; i<state_.size(); ++i)
        anglesOut[i-6] = state_(i);
    controlOut["joints"].setValues(anglesOut);

    // Update position of free flyer
    for (int i = 0;i < 3; ++i)
        baseff_[i*4+3] = freeFlyerPose () (i, 3);
    for(unsigned i = 0;i < 3; ++i)
        for(unsigned j = 0; j < 3; ++j)
            baseff_[i * 4 + j] = freeFlyerPose () (i, j);
    controlOut["baseff"].setValues(baseff_);

    sotDEBUGOUT(25);
}

void
Pr2Device::updateRobotState(const std::vector<double> &anglesIn)
{
    sotDEBUGIN(25);
    for (unsigned i=0; i<anglesIn.size(); ++i)
        mlRobotState(i+6) = anglesIn[i];
    robotState_.setConstant(mlRobotState);
    sotDEBUGOUT(25);
}


DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(Pr2Device,"Pr2Device");

}
