#ifndef PR2_DEVICE_H
#define PR2_DEVICE_H

#include <sot/core/device.hh>
#include <dynamic-graph/linear-algebra.h>
#include <sot/core/abstract-sot-external-interface.hh>

namespace sot_pr2 {

    typedef std::map<std::string, dynamicgraph::sot::SensorValues> SensorMap;
    typedef std::map<std::string, dynamicgraph::sot::ControlValues> ControlMap;

    class Pr2Device : public dynamicgraph::sot::Device {
        DYNAMIC_GRAPH_ENTITY_DECL();
    public:
        static const double TIMESTEP_DEFAULT;

    public:
        Pr2Device(const std::string &name);
        virtual ~Pr2Device();

        void setSensors(SensorMap &sensorsIn);

        void setupSetSensors(SensorMap &sensorsIn);
        void nominalSetSensors(SensorMap &sensorsIn);
        void cleanupSetSensors(SensorMap &sensorsIn);

        void getControl(ControlMap &controlOut);
        void getFakeControl(ControlMap &controlOut);

    protected:
        void updateRobotState(const std::vector<double> &anglesIn);

    protected:
        double timestep_;
        dynamicgraph::Vector previous_state_;
        dynamicgraph::Signal<dynamicgraph::Vector, int> robotState_;

    private:
        dynamicgraph::Vector mlRobotState;
        dynamicgraph::sot::MatrixRotation pose;
        std::vector<double> baseff_;

        int loop_count_;
    };
}

#endif
