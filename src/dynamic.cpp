#include <boost/assign.hpp>
#include <jrl/mal/matrixabstractlayer.hh>

#include <sot/core/debug.hh>

#include <abstract-robot-dynamics/humanoid-dynamic-robot.hh>
#include <abstract-robot-dynamics/robot-dynamics-object-constructor.hh>

#include <dynamic-graph/factory.h>

#include <jrl/dynamics/urdf/parser.hh>

#include "dynamic.hh"


namespace sot
{
  namespace pr2
  {
    namespace command
    {
      Load::Load (Dynamic& entity, const std::string& docstring)
	: Command (entity, boost::assign::list_of(Value::STRING), docstring)
      {}
      
      Value Load::doExecute()
      {
	Dynamic& entity =
	  static_cast<Dynamic&>(owner ());

	const std::vector<Value>& values = getParameterValues ();
	std::string filename = values[0].value ();

	entity.load (filename);
	return Value ();
      }
    } // end of namespace command.
    Dynamic::Dynamic(const std::string& name)
      : Entity(name),
	humanoidDynamicRobot_ (0)
    {
      std::string docstring = "";
      
      docstring =
	"";
      addCommand ("load", new command::Load (*this, docstring));
    }
    
    Dynamic::~Dynamic()
    {
      delete humanoidDynamicRobot_;
    }

    void
    Dynamic::load (const std::string& filename)
    {
      // load using jrl dynamics urdf.
    }

    DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(Dynamic, "DynamicPr2");
  } // end of namespace pr2.
} // end of namespace sot.
