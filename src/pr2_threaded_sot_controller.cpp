#include "sot_pr2/pr2_threaded_sot_controller.h"
#include <pluginlib/class_list_macros.h>
#include <dynamic_graph_bridge/ros_init.hh>

#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>

#include <sot/core/debug.hh>
#include <sot/core/exception-abstract.hh>

#include <unistd.h>

#define ROBOTNAME std::string("PR2")

namespace sot_pr2 {

const std::string Pr2ThreadedSotController::LOG_PYTHON="/tmp/pr2_sot_controller.out";
#define LOG_TRACE(x) sotDEBUG(25) << __FILE__ << ":" << __FUNCTION__ <<"(#" << __LINE__ << " ) " << x << std::endl

boost::condition_variable pr2_cond;
boost::condition cond2;
boost::mutex pr2_mut;
boost::mutex wait_start;
boost::mutex rmut, wmut;
bool pr2_data_ready;

/*struct timeval t0, t1;
int iter;*/

void
workThread(Pr2ThreadedSotController *actl) {
    dynamicgraph::Interpreter aLocalInterpreter(actl->node_);
    actl->interpreter_ = boost::make_shared<dynamicgraph::Interpreter>(aLocalInterpreter);

    std::cout << "Going through the thread." << std::endl;
    {
        boost::lock_guard<boost::mutex> lock(pr2_mut);
        pr2_data_ready=true;
    }
    pr2_cond.notify_all();
    ros::waitForShutdown();
}

void
sampleAndHold(Pr2ThreadedSotController *actl) {
    SensorMap deviceIn;
    ControlMap deviceOut;

    // Wait the start flag
    boost::unique_lock<boost::mutex> lock(wait_start);
    cond2.wait(lock);

    // Go go go !!!
    while (true) {
        {
            boost::mutex::scoped_lock lock(rmut);
            deviceIn = actl->holdIn();
        }
        actl->device()->nominalSetSensors(deviceIn);
        try {
            LOG_TRACE("");
            actl->device()->getControl(deviceOut);
            LOG_TRACE("");
        }
        catch (dynamicgraph::sot::ExceptionAbstract &err) {
            LOG_TRACE(err.getStringMessage());
            throw err;
        }
        actl->device()->getControl(deviceOut);
        {
            boost::mutex::scoped_lock lock(wmut);
            actl->holdOut() = deviceOut;
        }
        usleep(1);
        //boost::this_thread::sleep(boost::posix_time::nanoseconds(1000));
    }
}

Pr2ThreadedSotController::Pr2ThreadedSotController()
: node_(dynamicgraph::rosInit(false,true))
, device_(ROBOTNAME)
{
    std::cout << "Going through Pr2ThreadedSotController." << std::endl;
    boost::thread thr(workThread,this);
    LOG_TRACE("");
    boost::unique_lock<boost::mutex> lock(pr2_mut);
    pr2_cond.wait(lock);
    startupPython();
    interpreter_->startRosService ();
}

Pr2ThreadedSotController::~Pr2ThreadedSotController() {
}

void
Pr2ThreadedSotController::init() {
    boost::thread sampleAndHoldProcess(sampleAndHold,this);
    //sampleAndHoldProcess.join();
}

void
Pr2ThreadedSotController::setupSetSensors(SensorMap &sensorsIn) {
    /*gettimeofday(&t0, 0);
    iter = 0;*/
    //device_.setupSetSensors(sensorsIn);
    {
        boost::mutex::scoped_lock lock(rmut);
        _holdIn = sensorsIn;
    }
    cond2.notify_all();
}

void
Pr2ThreadedSotController::nominalSetSensors(SensorMap &sensorsIn) {
    /*++iter;
    gettimeofday(&t1, 0);
    long elapsed = (t1.tv_sec-t0.tv_sec)*1000000 + t1.tv_usec-t0.tv_usec;
    if (elapsed >= 1000000) {
        double elapsed_s = elapsed / 1000000;
        //std::cout << "[Pr2ThreadedSotController] " << iter / elapsed_s << " Hz" << std::endl;
        t0 = t1;
        iter = 0;
    }*/
    //device_.nominalSetSensors(sensorsIn);
    {
        boost::mutex::scoped_lock lock(rmut);
        _holdIn = sensorsIn;
    }
}

void
Pr2ThreadedSotController::cleanupSetSensors(SensorMap &sensorsIn) {
    //device_.cleanupSetSensors(sensorsIn);
    {
        boost::mutex::scoped_lock lock(rmut);
        _holdIn = sensorsIn;
    }
}

void
Pr2ThreadedSotController::getControl(ControlMap &controlOut) {
    /*try {
        LOG_TRACE("");
        device_.getControl(controlOut);
        LOG_TRACE("");
    }
    catch (dynamicgraph::sot::ExceptionAbstract &err) {
        LOG_TRACE(err.getStringMessage());
        throw err;
    }*/
    {
        boost::mutex::scoped_lock lock(wmut);
        controlOut = _holdOut;
    }
}


void
Pr2ThreadedSotController::runPython(std::ostream &file,
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
Pr2ThreadedSotController::startupPython() {
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

/*extern "C"
{
  dynamicgraph::sot::AbstractSotExternalInterface * createSotExternalInterface()
  {
    return new sot_pr2::Pr2ThreadedSotController;
  }
}

extern "C"
{
  void destroySotExternalInterface(dynamicgraph::sot::AbstractSotExternalInterface *p)
  {
    delete p;
  }
}


*/
