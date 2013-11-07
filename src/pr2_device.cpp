#include "sot_pr2/pr2_device.h"

#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>

namespace sot_pr2 {

const double Pr2Device::TIMESTEP_DEFAULT = 0.001;

Pr2Device::Pr2Device(const std::string &name)
: dynamicgraph::sot::Device(name),
  timestep_(TIMESTEP_DEFAULT) {
}

Pr2Device::~Pr2Device() {
}


bool
Pr2Device::init() {
    return true;
}

void
Pr2Device::setup(jointMap_t &jm) {
    int t = stateSOUT.getTime() + 1;
    maal::boost::Vector state = stateSOUT.access(t);

    sotDEBUG (25) << "stateSOUT.access (" << t << ") = " << state << std::endl;
    sotDEBUG (25) << "state.size () = " << state.size () << std::endl;
    sotDEBUG (25) << "joints_.size () = " << jm.size () << std::endl;

    unsigned jointId = 0;
    jointMap_t::const_iterator it;
    for (it=jm.begin(); it!=jm.end(); ++it, ++jointId) {
        if (jointId+6 >= state.size() || !it->second.second)
            continue;
        state(jointId+6) = it->second.second->position_;
    }

    ROS_INFO_STREAM("state : " << state);

    stateSOUT.setConstant(state);
    state_ = state;
}

void
Pr2Device::control(jointMap_t &jm) {
    try {
        increment(timestep_);
    }
    catch (...) {}

    sotDEBUG (25) << "state = " << state_ << std::endl;

    unsigned jointId = 0;
    jointMap_t::const_iterator it;
    for (it=jm.begin(); it!=jm.end(); ++it, ++jointId) {
        if (jointId+6 >= state_.size() || !it->second.second)
            continue;
        it->second.second->commanded_effort_ = state_(jointId+6);
    }
}

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(Pr2Device,"Pr2Device");

}
