#ifndef SOT_SOT_PR2_HH
# define SOT_SOT_PR2_HH
# include <pr2_controller_interface/controller.h>
# include <pr2_mechanism_model/joint.h>

# include <dynamic_graph_bridge/ros_interpreter.hh>
# include "pr2.hh"

# include <sot/core/device.hh>

namespace sot_pr2
{
  class SotPr2 : public pr2_controller_interface::Controller
  {
  private:
    /// Embedded python interpreter accessible via a ROS service.
    dynamicgraph::Interpreter interpreter_;
    jointMap_t jointsMap_;
    /// Pointer to Entity StackOfTasks
    Pr2* entity_;
  public:
    explicit SotPr2();

    virtual bool init(pr2_mechanism_model::RobotState* robot,
		      ros::NodeHandle &n);
    virtual void starting();
    virtual void update();
    virtual void stopping();
  };
} // end of namespace sot_pr2

#endif //! SOT_SOT_PR2_HH
