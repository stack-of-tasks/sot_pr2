#ifndef SOT_PR2_HH
# define SOT_PR2_HH
# include <sot/core/device.hh>

# include <urdf_interface/joint.h>
# include <pr2_controller_interface/controller.h>
# include <pr2_mechanism_model/joint.h>


namespace sot_pr2
{
  typedef
  std::map<std::string,
	   std::pair<boost::shared_ptr<urdf::Joint>,
		     boost::shared_ptr<pr2_mechanism_model::JointState> > >
    jointMap_t;

  class Pr2 : public dynamicgraph::sot::Device
  {
    DYNAMIC_GRAPH_ENTITY_DECL();
  public:
    /// \brief Default constructor
    ///
    /// \param entityName entity name in the dynamic-graph pool
    Pr2(const std::string& entityName);
    virtual ~Pr2();

    /// \name Inherited control methods.
    /// \{

    /// \brief Called when plug-in is started.
    virtual bool setup (jointMap_t& jointMap);

    /// \brief Called at each control loop.
    virtual void control (jointMap_t& jointMap);

    /// \brief Called when plug-in is stopped.
    virtual bool cleanup (jointMap_t& jointMap);

    /// \}

  protected:
    void updateRobotState (jointMap_t& jointMap);

  private:
    /// \brief Default integration step (i.e. 1ms for PR2).
    static const double TIMESTEP_DEFAULT;

    double timestep_;
    ml::Vector previousState_;
    dynamicgraph::Signal<ml::Vector, int> robotState_;
  };
} // end of namespace sot_pr2

#endif //! SOT_PR2_HH
