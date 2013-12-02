#ifndef PR2_CONTROLLER_PLUGIN_H
#define PR2_CONTROLLER_PLUGIN_H

#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/joint.h>
#include <sot_pr2/pr2_sot_controller.h>
#include <control_toolbox/pid.h>
#include <boost/scoped_ptr.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <tf/transform_listener.h>

namespace sot_pr2 {

typedef boost::shared_ptr<pr2_mechanism_model::JointState> Pr2JointPtr;

class Pr2ControllerPlugin : public pr2_controller_interface::Controller {
public:
    explicit Pr2ControllerPlugin();
    virtual ~Pr2ControllerPlugin();

    virtual bool init(pr2_mechanism_model::RobotState *robot,
                      ros::NodeHandle &n);
    virtual void starting();
    virtual void update();
    virtual void stopping();

private:
    void fillSensors();
    void readControl();

private:
    // SoT Controller
    Pr2SotController sot_controller_;
    SensorMap sensorsIn_;
    ControlMap controlValues_;

    std::vector<double> joint_encoder_;
    std::vector<double> joint_control_;
    std::vector<double> error_raw;
    std::vector<double> error;


    // Pr2 Controller
    int loop_count_;
    ros::Time last_time_;
    std::vector<Pr2JointPtr> joints_;
    std::vector<control_toolbox::Pid> pids_;
    pr2_mechanism_model::RobotState *robot_;

    // ROS interface
    //ros::NodeHandle node_;
    boost::scoped_ptr<
        realtime_tools::RealtimePublisher<
            control_msgs::JointTrajectoryControllerState> > controller_state_publisher_;

    ros::Publisher cmd_vel_pub_;

    tf::TransformListener listener_;

    double timeFromStart_;
};

}

#endif
