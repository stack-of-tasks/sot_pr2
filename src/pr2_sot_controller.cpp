#include "sot_pr2/pr2_sot_controller.h"
#include <pluginlib/class_list_macros.h>
#include <dynamic_graph_bridge/ros_init.hh>

#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>

#include <sot/core/debug.hh>
#include <sot/core/exception-abstract.hh>

#define ROBOTNAME std::string("PR2")

namespace sot_pr2 {

const std::string Pr2SotController::LOG_PYTHON="/tmp/pr2_sot_controller.out";
#define LOG_TRACE(x) sotDEBUG(25) << __FILE__ << ":" << __FUNCTION__ <<"(#" << __LINE__ << " ) " << x << std::endl

boost::condition_variable cond;
boost::mutex mut;
bool data_ready;

void
workThread(Pr2SotController *actl) {
    dynamicgraph::Interpreter aLocalInterpreter(actl->node_);
    actl->interpreter_ = boost::make_shared<dynamicgraph::Interpreter>(aLocalInterpreter);

    std::cout << "Going through the thread." << std::endl;
    {
        boost::lock_guard<boost::mutex> lock(mut);
        data_ready=true;
    }
    cond.notify_all();
    ros::waitForShutdown();
}

Pr2SotController::Pr2SotController()
: node_(dynamicgraph::rosInit(false,true))
, device_(ROBOTNAME)
{
    std::cout << "Going through Pr2SotController." << std::endl;
    boost::thread thr(workThread,this);
    LOG_TRACE("");
    boost::unique_lock<boost::mutex> lock(mut);
    cond.wait(lock);
    startupPython();
    interpreter_->startRosService ();
}

Pr2SotController::~Pr2SotController() {
}

void
Pr2SotController::setupSetSensors(SensorMap &sensorsIn) {
    device_.setupSetSensors(sensorsIn);
}

void
Pr2SotController::nominalSetSensors(SensorMap &sensorsIn) {
    device_.nominalSetSensors(sensorsIn);
}

void
Pr2SotController::cleanupSetSensors(SensorMap &sensorsIn) {
    device_.cleanupSetSensors(sensorsIn);
}

void
Pr2SotController::getControl(ControlMap &controlOut) {
    try {
        LOG_TRACE("");
        device_.getControl(controlOut);
        LOG_TRACE("");
    }
    catch (dynamicgraph::sot::ExceptionAbstract &err) {
        LOG_TRACE(err.getStringMessage());
        throw err;
    }
}


void
Pr2SotController::runPython(std::ostream &file,
                            const std::string &command,
                            dynamicgraph::Interpreter &interpreter) {
    file << ">>> " << command << std::endl;
    std::string lerr(""),lout(""),lres("");
    interpreter.runCommand(command,lres,lout,lerr);
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

void
Pr2SotController::startupPython() {
    std::ofstream aof(LOG_PYTHON.c_str());
    runPython (aof, "import sys, os", *interpreter_);
    runPython (aof, "pythonpath = os.environ['PYTHONPATH']", *interpreter_);
    runPython (aof, "path = []", *interpreter_);
    runPython (aof, "for p in pythonpath.split(':'):\n"
                    "  if p not in sys.path:\n"
                    "    path.append(p)", *interpreter_);
    runPython (aof, "path.extend(sys.path)", *interpreter_);
    runPython (aof, "sys.path = path", *interpreter_);
    runPython (aof, "from dynamic_graph.sot.pr2.prologue import robot", *interpreter_);

    dynamicgraph::rosInit(true);

    aof.close();
}

}

extern "C"
{
  dynamicgraph::sot::AbstractSotExternalInterface * createSotExternalInterface()
  {
    return new sot_pr2::Pr2SotController;
  }
}

extern "C"
{
  void destroySotExternalInterface(dynamicgraph::sot::AbstractSotExternalInterface *p)
  {
    delete p;
  }
}


