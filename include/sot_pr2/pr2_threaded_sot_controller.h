#ifndef PR2_SOT_CONTROLLER_H
#define PR2_SOT_CONTROLLER_H

#include <sot_pr2/pr2_device.h>
#include <dynamic_graph_bridge/ros_interpreter.hh>
#include <sot/core/abstract-sot-external-interface.hh>

namespace sot_pr2 {

class Pr2ThreadedSotController : public dynamicgraph::sot::AbstractSotExternalInterface
{
public:
    static const std::string LOG_PYTHON;

public:
    explicit Pr2ThreadedSotController();
    virtual ~Pr2ThreadedSotController();

    void init();

    void setupSetSensors(SensorMap &sensorsIn);
    void nominalSetSensors(SensorMap &sensorsIn);
    void cleanupSetSensors(SensorMap &sensorsIn);

    void getControl(ControlMap &controlOut);

    boost::shared_ptr<dynamicgraph::Interpreter> interpreter_;
    ros::NodeHandle node_;

protected:
    void runPython(std::ostream &file,
                   const std::string &command,
                   dynamicgraph::Interpreter &interpreter);

    virtual void startupPython();

private:
    Pr2Device device_;

    SensorMap _holdIn;
    ControlMap _holdOut;

public:
    Pr2Device *device() {return &device_;}
    SensorMap &holdIn() {return _holdIn;}
    ControlMap &holdOut() {return _holdOut;}
};

}

#endif
