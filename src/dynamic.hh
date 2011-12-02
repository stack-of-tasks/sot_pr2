#ifndef SOT_PR2_DYNAMIC_HH
# define SOT_PR2_DYNAMIC_HH
# include <string>

# include <jrl/mal/boost.hh>
# include "jrl/mal/matrixabstractlayer.hh"

namespace ml = maal::boost;

# include <abstract-robot-dynamics/humanoid-dynamic-robot.hh>
# include <jrl/dynamics/dynamicsfactory.hh>

# include <sot/core/flags.hh>
# include <dynamic-graph/command.h>
# include <dynamic-graph/entity.h>
# include <dynamic-graph/pool.h>
# include <dynamic-graph/signal-ptr.h>
# include <dynamic-graph/signal-time-dependent.h>
# include <sot/core/exception-dynamic.hh>
# include <sot/core/matrix-homogeneous.hh>

namespace sot
{
  namespace dg = dynamicgraph;

  namespace pr2
  {
    class Dynamic;

    namespace command
    {
      using ::dynamicgraph::command::Command;
      using ::dynamicgraph::command::Value;

      class Load : public Command
      {
      public:
	explicit Load (Dynamic& entity, const std::string& docstring);
	Value doExecute();
      };
    } // end of namespace command.


    class Dynamic : public dg::Entity
    {
      DYNAMIC_GRAPH_ENTITY_DECL ();
    public:
      Dynamic(const std::string& name);
      virtual ~Dynamic();

      void load (const std::string& filename);
    private:
      CjrlHumanoidDynamicRobot* humanoidDynamicRobot_;
    };
  } // end of namespace pr2.
} // end of namespace sot.


#endif //! SOT_PR2_DYNAMIC_HH
