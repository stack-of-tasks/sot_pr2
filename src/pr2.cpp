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
    robotState_ ("Pr2(" + entityName + ")::output(vector)::robotState"),
    pids_ (),
    torqueControl_ (false),
    lastTime_ ()
{
  signalRegistration (robotState_);
}

bool
Pr2::init (ros::NodeHandle& nh,
	   pr2_mechanism_model::RobotState* robot,
	   jointMap_t& jointMap)
{
  robot_ = robot;

  // Retrieve parameters.
  ros::param::param<bool> ("~torque_control", torqueControl_, false);

  // If PID used, reset it.
  if (!torqueControl_)
    for (jointMap_t::iterator it = jointMap.begin ();
	 it != jointMap.end (); ++it)
      {
	boost::shared_ptr<control_toolbox::Pid> pid =
	  boost::make_shared<control_toolbox::Pid> ();
	if (!pid->init (ros::NodeHandle (nh, "pid_parameters")))
	  {
	    ROS_ERROR ("failed to construct PID controller");
	    return false;
	  }
	pid->reset ();
	pids_.push_back (pid);
      }
  return true;
}

bool
Pr2::setup (jointMap_t& jointMap)
{
  // Reset PID if necessary.
  for (unsigned i = 0; i < pids_.size (); ++i)
    if (pids_[i])
      pids_[i]->reset ();

  lastTime_ = robot_->getTime ();

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
    {
      if (jointId + 6 >= state.size ()
	  || !it->second.second)
	continue;
      state (jointId + 6) = it->second.second->position_;
    }

  previousState_ = state;
  stateSOUT.setConstant (state);

  updateRobotState (jointMap);
  return true;
}

void
Pr2::control (jointMap_t& jointMap)
{
  // Integrate control
  try
    {
      increment (timestep_);
    }
  catch (...)
    {}
  sotDEBUG (25) << "state = " << state_ << std::endl;
  sotDEBUG (25) << "diff  = " << state_ - previousState_ << std::endl;

  // Write new state into motor command.
  unsigned jointId = 0;
  if (!pids_.empty ())
    for (jointMap_t::iterator it = jointMap.begin ();
	 it != jointMap.end ();
	 ++it, ++jointId)
      {
	ros::Duration dt = robot_->getTime () - lastTime_;
	lastTime_ = robot_->getTime ();
	if (jointId + 6 >= state_.size ()
	    || jointId + 6 >= previousState_.size ()
	    || !it->second.second)
	  continue;
	it->second.second->commanded_effort_ =
	  pids_[jointId]->updatePid
	  (previousState_ (jointId + 6) - state_ (jointId + 6),
	   dt);
      }
  else
    for (jointMap_t::iterator it = jointMap.begin ();
	 it != jointMap.end ();
	 ++it, ++jointId)
      {
	if (jointId + 6 >= state_.size ()
	    || jointId + 6 >= previousState_.size ())
	  continue;
	it->second.second->commanded_effort_ = state_ (jointId + 6);
      }

  previousState_ = state_;
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
    {
      if (jointId + 6 >= robotState.size ()
	  || !it->second.second)
	continue;
      robotState (jointId + 6) = it->second.second->position_;
    }
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
