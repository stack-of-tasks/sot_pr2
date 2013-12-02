#include "sot_pr2/pr2_controller_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Twist.h>

namespace sot_pr2 {

std::ofstream logout;

Pr2ControllerPlugin::Pr2ControllerPlugin()
    : pr2_controller_interface::Controller(),
      sot_controller_("SoTPR2"),
      loop_count_(0),
      robot_(NULL) {
    logout.open("/tmp/out.log", std::ios::out);
}

Pr2ControllerPlugin::~Pr2ControllerPlugin() {
}

bool
Pr2ControllerPlugin::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n) {
    sot_controller_.node_ = n;
    cmd_vel_pub_ = sot_controller_.node_.advertise<geometry_msgs::Twist>("/base_controller/command", 1);

    // Check initialization
    if (!robot) {
        ROS_ERROR_STREAM("NULL robot pointer");
        return false;
    }
    if (!robot->model_) {
        ROS_ERROR_STREAM("NULL model pointer");
        return false;
    }
    robot_ = robot;

    // Get the joints
    XmlRpc::XmlRpcValue joint_names;
    if (!sot_controller_.node_.getParam("joints", joint_names)) {
        ROS_ERROR("No joints given. (namespace: %s)", sot_controller_.node_.getNamespace().c_str());
        return false;
    }
    if (joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR("Malformed joint specification. (namespace: %s)", sot_controller_.node_.getNamespace().c_str());
        return false;
    }
    for (int i=0; i<joint_names.size(); ++i) {
        XmlRpc::XmlRpcValue &name_value = joint_names[i];
        if (name_value.getType() != XmlRpc::XmlRpcValue::TypeString) {
            ROS_ERROR("Array of joint names should contain all strings. (namespace: %s)", sot_controller_.node_.getNamespace().c_str());
            return false;
        }
        Pr2JointPtr j;
        j.reset(robot->getJointState((std::string)name_value));
        if (!j) {
            ROS_ERROR("Joint not found: %s. (namespace: %s)", ((std::string)name_value).c_str(), sot_controller_.node_.getNamespace().c_str());
            return false;
        }
        joints_.push_back(j);
    }

    // Ensures joints are calibrated
    for (size_t i=0; i<joints_.size(); ++i) {
        if (!joints_[i]->calibrated_) {
            ROS_ERROR("Joint %s was not calibrated (namespace: %s)", joints_[i]->joint_->name.c_str(), sot_controller_.node_.getNamespace().c_str());
            return false;
        }
    }

    // Setup PID controllers
    std::string gains_ns;
    if (!sot_controller_.node_.getParam("gains", gains_ns))
        gains_ns = sot_controller_.node_.getNamespace() + "/gains";
    pids_.resize(joints_.size());
    for (size_t i=0; i<joints_.size(); ++i) {
        if (!pids_[i].init(ros::NodeHandle(gains_ns + "/" + joints_[i]->joint_->name))) {
            if (!pids_[i].init(ros::NodeHandle(sot_controller_.node_,"pid_parameters"))) {
                ROS_ERROR("Failed to build PID controller");
                return false;
            }
        }
    }

    // TF Listener
    listener_.waitForTransform("base_footprint", "odom_combined", ros::Time(0), ros::Duration(1.0));

    // Allocate space
    const unsigned int jsz = joints_.size();
    joint_encoder_.resize(jsz);
    joint_control_.resize(jsz);
    error_raw.resize(jsz);
    error.resize(jsz);

    controller_state_publisher_.reset(
        new realtime_tools::RealtimePublisher<control_msgs::JointTrajectoryControllerState>
                (sot_controller_.node_, "state", 1));
    controller_state_publisher_->lock();
    for (size_t j=0; j<joints_.size(); ++j)
        controller_state_publisher_->msg_.joint_names.push_back(joints_[j]->joint_->name);
    controller_state_publisher_->msg_.desired.positions.resize(joints_.size());
    controller_state_publisher_->msg_.desired.velocities.resize(joints_.size());
    controller_state_publisher_->msg_.desired.accelerations.resize(joints_.size());
    controller_state_publisher_->msg_.actual.positions.resize(joints_.size());
    controller_state_publisher_->msg_.actual.velocities.resize(joints_.size());
    controller_state_publisher_->msg_.error.positions.resize(joints_.size());
    controller_state_publisher_->msg_.error.velocities.resize(joints_.size());
    controller_state_publisher_->unlock();

    timeFromStart_ = 0.0;

    return true;
}

void
Pr2ControllerPlugin::fillSensors() {
    // Joint values
    sensorsIn_["joints"].setName("position");
    for (unsigned int i=0; i<joints_.size(); ++i)
        joint_encoder_[i] = joints_[i]->position_;
    sensorsIn_["joints"].setValues(joint_encoder_);
}

void
Pr2ControllerPlugin::readControl() {
    ros::Time time = robot_->getTime();
    ros::Duration dt = time - last_time_;
    last_time_ = time;

    // Update command
    joint_control_ = controlValues_["joints"].getValues();
    for (unsigned int i=0; i<joints_.size(); ++i) {
        error_raw[i] = joints_[i]->position_ - joint_control_[i];
        if(error_raw[i] < -3.14159){
            error[i] = error_raw[i] + 3.14159*2;
        } else {
            if(error_raw[i] > 3.14159){
                error[i] = error_raw[i] - 3.14159*2;
            } else {
                error[i] = error_raw[i];
            }
        }
        joints_[i]->commanded_effort_ += pids_[i].updatePid(error[i], dt);
    }

    // Get Odometry
    tf::StampedTransform current_transform;
    listener_.lookupTransform("base_footprint", "odom_combined", ros::Time(0), current_transform);
    tf::Vector3 xyz = current_transform.getOrigin();
    tf::Quaternion q = current_transform.getRotation();
    std::cout << "Current pos : " << std::endl
              << xyz[0] << ", "  << xyz[1] << ", " << xyz[2] << std::endl
              << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << std::endl;
    double curr_yaw = std::atan2(2*(q.w()*q.z() + q.x()*q.y()), 1 - 2*(q.y()*q.y() + q.z()*q.z()));
    std::cout << "yaw = " << curr_yaw << std::endl;

    // Base controller
    geometry_msgs::Twist base_cmd;
    std::vector<double> baseff = controlValues_["baseff"].getValues();
    double yaw = std::atan2(baseff[4],baseff[0]);
    double pitch = std::atan2(-baseff[8],std::sqrt(baseff[9]*baseff[9] + baseff[10]*baseff[10]));
    double roll = std::atan2(baseff[9],baseff[10]);
    std::cout << "baseff = [" << baseff[3] << ", " << baseff[7] << ", " << baseff[11] << ", "
                              << roll << ", " << pitch << ", " << yaw << "]" << std::endl;
    base_cmd.linear.x = base_cmd.linear.y = base_cmd.linear.z = 0;
    base_cmd.angular.x = base_cmd.angular.y = base_cmd.angular.z = 0;
    base_cmd.linear.x = 1.0 * (baseff[3] - xyz[0]);
    base_cmd.linear.y = 1.0 * (baseff[7] - xyz[1]);
    base_cmd.angular.z = 1.0 * (yaw - curr_yaw);
    cmd_vel_pub_.publish(base_cmd);

    logout << xyz[0] << "\t" << baseff[3] << "\t" << base_cmd.linear.x << std::endl;

    // State publishing
    if (loop_count_ % 10 == 0) {
        if (controller_state_publisher_ && controller_state_publisher_->trylock()) {
            controller_state_publisher_->msg_.header.stamp = time;
            for (size_t j=0; j<joints_.size(); ++j) {
                controller_state_publisher_->msg_.desired.positions[j] = joint_control_[j];
                controller_state_publisher_->msg_.actual.positions[j] = joints_[j]->position_;
                controller_state_publisher_->msg_.actual.velocities[j] = joints_[j]->velocity_;
                controller_state_publisher_->msg_.actual.time_from_start= ros::Duration(timeFromStart_);
                controller_state_publisher_->msg_.error.positions[j] = error[j];
            }
            controller_state_publisher_->unlockAndPublish();
        }
    }
    ++loop_count_;
}


void
Pr2ControllerPlugin::starting() {
    std::cout << "STARTING" << std::endl;
    last_time_ = robot_->getTime();

    for (size_t i=0; i<pids_.size(); ++i)
        pids_[i].reset();

    fillSensors();
    try {
        sot_controller_.setupSetSensors(sensorsIn_);
        sot_controller_.getControl(controlValues_);
    }
    catch (std::exception &e) { throw e; }
    readControl();
}

void
Pr2ControllerPlugin::update() {
    fillSensors();
    try {
        sot_controller_.nominalSetSensors(sensorsIn_);
        sot_controller_.getControl(controlValues_);
    }
    catch (std::exception &e) { throw e; }
    readControl();
}

void
Pr2ControllerPlugin::stopping() {
    fillSensors();
    try {
        sot_controller_.cleanupSetSensors(sensorsIn_);
        sot_controller_.getControl(controlValues_);
    }
    catch (std::exception &e) { throw e; }
    readControl();
}

/// Register controller to pluginlib
PLUGINLIB_EXPORT_CLASS(sot_pr2::Pr2ControllerPlugin,
                       pr2_controller_interface::Controller)

}
