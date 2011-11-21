#include "pr2.hh"
#include "sot_pr2.hh"
#include <pluginlib/class_list_macros.h>

const std::string SOT_OPENHRP_OUTPUT_FILE ("/tmp/sot.out");

namespace sot_pr2
{
  static void
  runPython (std::ostream& file,
	     const std::string& command,
	     dynamicgraph::corba::Interpreter& interpreter)
  {
    file << ">>> " << command << std::endl;
    std::string value = interpreter.python (command);
    if (value != "None")
      file << value;
  }

  SotPr2::SotPr2()
    : pr2_controller_interface::Controller (),
      jointsMap_ (),
      interpreter_ (),
      entity_ (new Pr2 ("robot_device"))
  {}
      

  /// Controller initialization in non-realtime
  bool SotPr2::init(pr2_mechanism_model::RobotState* robot,
		    ros::NodeHandle& n)
  {
    if (!robot)
      {
	ROS_ERROR_STREAM("null robot pointer");
	return false;
      }

    // Fill joint map.
    if (!robot->model_)
      {
	ROS_ERROR_STREAM("null model pointer");
	return false;
      }

    typedef std::map<std::string, boost::shared_ptr<urdf::Joint> >::const_iterator
      iter_t;
    for (iter_t it = robot->model_->robot_model_.joints_.begin();
	 it != robot->model_->robot_model_.joints_.end ();
	 ++it)
      {
	boost::shared_ptr<pr2_mechanism_model::JointState> state 
	  (robot->getJointState (it->first));
	jointsMap_[it->first] = std::make_pair(it->second, state);
      }

    // Call prologue.
    try
      {
	std::ofstream aof (SOT_OPENHRP_OUTPUT_FILE.c_str ());
	runPython (aof, "import sys, os", interpreter_);
	runPython (aof, "pythonpath = os.environ['PYTHONPATH']", interpreter_);
	runPython (aof, "path = []", interpreter_);
	runPython (aof,
		   "for p in pythonpath.split(':'):\n"
		   "  if p not in sys.path:\n"
		   "    path.append(p)", interpreter_);
	runPython (aof, "path.extend(sys.path)", interpreter_);
	runPython (aof, "sys.path = path", interpreter_);
	runPython
	  (aof,
	   "from dynamic_graph.sot.pr2.prologue import robot, solver",
	   interpreter_);
	interpreter_.startCorbaServer ("pr2", "", "stackOfTasks", "");
      }
    catch(const std::exception& e)
      {
	ROS_ERROR_STREAM("failed to initialize controller: " << e.what());
	return false;
      }
    catch(...)
      {
	ROS_ERROR_STREAM
	  ("unknown exception catched during controller initialization");
	return false;
      }
    return true;
  }


  /// Controller startup in realtime
  void SotPr2::starting()
  {
    if (!entity_->setup(jointsMap_))
      ROS_ERROR_STREAM("failed to start");
  }


  /// Controller update loop in realtime
  void SotPr2::update()
  {
    entity_->control (jointsMap_);
  }


  /// Controller stopping in realtime
  void SotPr2::stopping()
  {
    if (!entity_->cleanup (jointsMap_))
      ROS_ERROR_STREAM("failed to stop");
  }

} // end of namespace sot_pr2.

/// Register controller to pluginlib
PLUGINLIB_DECLARE_CLASS(sot_pr2,SotPr2Plugin,
			sot_pr2::SotPr2,
			pr2_controller_interface::Controller)
