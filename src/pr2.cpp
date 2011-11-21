#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/debug.h>
#include <sot/core/debug.hh>
#include <sot/core/exception-factory.hh>
#include <dynamic-graph/all-commands.h>

#include "pr2.hh"

#include <dynamic-graph/all-commands.h>
#include "sot/core/api.hh"

using sot_pr2::Pr2;
using dynamicgraph::sot::ExceptionFactory;

const double Pr2::TIMESTEP_DEFAULT = 0.001;
const std::string Pr2::CLASS_NAME = "Pr2";

Pr2::Pr2 (const std::string& entityName)
  : dynamicgraph::sot::Device (entityName),
    timestep_ (TIMESTEP_DEFAULT),
    previousState_ (),
    robotState_ ("Pr2(" + entityName + ")::output(vector)::robotState")
{
  signalRegistration (robotState_);
}

bool
Pr2::setup (jointMap_t& jointMap)
{
  // Read state from motor command
  int t = stateSOUT.getTime () + 1;
  maal::boost::Vector state = stateSOUT.access (t);

  sotDEBUG (25) << "stateSOUT.access (" << t << ") = " << state << std::endl;
  sotDEBUG (25) << "state.size () = " << state.size () << std::endl;
  sotDEBUG (25) << "jointMap.size () = " << jointMap.size () << std::endl;

  unsigned jointId = 0;
  for (jointMap_t::const_iterator it = jointMap.begin ();
       it != jointMap.end ();
       ++it, ++jointId)
    state (jointId + 6) = it->second.second->measured_effort_;

  previousState_ = state;
  stateSOUT.setConstant (state);

  updateRobotState (jointMap);
  return true;
}

void
Pr2::control (jointMap_t& jointMap)
{
  // Integrate control
  increment (timestep_);
  sotDEBUG (25) << "state = " << state_ << std::endl;
  sotDEBUG (25) << "diff  = " << state_ - previousState_ << std::endl;
  previousState_ = state_;

  // Write new state into motor command.
  unsigned jointId = 6;
  for (jointMap_t::iterator it = jointMap.begin ();
       it != jointMap.end ();
       ++it, ++jointId)
    it->second.second->commanded_effort_ = state_ (jointId - 6);

  updateRobotState (jointMap);
}

void
Pr2::updateRobotState (jointMap_t& jointMap)
{
  ml::Vector robotState (jointMap.size () + 6);
  for (unsigned i = 0; i < 6; ++i)
    robotState (i) = 0.;
  unsigned jointId = 0;
  for (jointMap_t::const_iterator it = jointMap.begin ();
       it != jointMap.end ();
       ++it, ++jointId)
    robotState (jointId + 6) = it->second.second->commanded_effort_;
  robotState_.setConstant(robotState);
}

bool
Pr2::cleanup (jointMap_t& jointMap)
{
  return true;
}

Pr2::~Pr2()
{
}
