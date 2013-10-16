#include "sot_pr2/pr2_sot_controller.h"
#include <pluginlib/class_list_macros.h>
#include <dynamic_graph_bridge/ros_init.hh>

namespace sot_pr2 {

const std::string Pr2SotController::LOG_PYTHON="/tmp/pr2_sot_controller.out";

Pr2SotController::Pr2SotController()
: pr2_controller_interface::Controller(),
  interpreter_(dynamicgraph::rosInit(false)),
  joint_map_(),
  device_(new Pr2Device("robot_device")) {
}

void Pr2SotController::runPython(std::ostream& file, const std::string& command)
{
    file << ">>> " << command << std::endl;
    std::string lerr(""),lout(""),lres("");
    interpreter_.runCommand(command,lres,lout,lerr);
    if (lres != "None") {
        if (lres=="<NULL>") {
            file << lout << std::endl;
            file << "------" << std::endl;
            file << lerr << std::endl;

            std::string err("Exception catched during sot controller initialization, please check the log file: " + LOG_PYTHON);
            throw std::runtime_error(err);
        }
        else
            file << lres << std::endl;
    }
}

/// Controller initialization in non-realtime
bool Pr2SotController::init(pr2_mechanism_model::RobotState *robot,
                            ros::NodeHandle &nh)
{
    // Check initialization
    if (!robot) {
        ROS_ERROR_STREAM("NULL robot pointer");
        return false;
    }

    if (!robot->model_) {
        ROS_ERROR_STREAM("NULL model pointer");
        return false;
    }

    // Fill joint map
    std::map<std::string, UrdfJointPtr>::const_iterator it;
    for (it=robot->model_->robot_model_.joints_.begin(); it!=robot->model_->robot_model_.joints_.end();++it) {
        Pr2JointPtr state(robot->getJointState(it->first));
        joint_map_[it->first] = std::make_pair(it->second, state);
    }

    // Init Device
    if (!device_->init()) {
        ROS_ERROR_STREAM("Device failed to initialize");
        return false;
    }

    // Bind with SoT
    try {
        std::ofstream aof(LOG_PYTHON.c_str());
        runPython (aof, "import sys, os");
        runPython (aof, "pythonpath = os.environ['PYTHONPATH']");
        runPython (aof, "path = []");
        runPython (aof, "for p in pythonpath.split(':'):\n"
                        "  if p not in sys.path:\n"
                        "    path.append(p)");
        runPython (aof, "path.extend(sys.path)");
        runPython (aof, "sys.path = path");
        runPython (aof, "from dynamic_graph.sot.pr2.prologue import robot, solver");

        interpreter_.startRosService ();
    }
    catch (const std::exception &e) {
        ROS_ERROR_STREAM("Failed to initialize controller: " << e.what());
        return false;
    }
    catch (...) {
        ROS_ERROR_STREAM("Failed to initialize controller: Unknown exception.");
        return false;
    }

    return true;
}


/// Controller startup in realtime
void Pr2SotController::starting()
{
    device_->setup(joint_map_);
}


/// Controller update loop in realtime
void Pr2SotController::update()
{
    device_->control(joint_map_);
}


/// Controller stopping in realtime
void Pr2SotController::stopping()
{
}

} // namespace

/// Register controller to pluginlib
PLUGINLIB_EXPORT_CLASS(sot_pr2::Pr2SotController,
                       pr2_controller_interface::Controller)
