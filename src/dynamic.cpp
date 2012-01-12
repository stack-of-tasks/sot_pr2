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
	  jrl::dynamics::urdf::Parser parser;
      humanoidDynamicRobot_ = parser.parse(filename, "base_footprint_joint");

      // iterate on tree nodes and add signals
      typedef dg::SignalTimeDependent<MatrixHomogeneous, int> signal_t;
      ::std::vector<CjrlJoint*> jointsVect = humanoidDynamicRobot_->jointVector();

      for (uint i=0; i<jointsVect.size(); ++i)
      {
	    CjrlJoint* currentJoint = jointsVect[i];
	    std::string signame = currentJoint->getName();

 		signal_t* sig
	    = new signal_t
	    (boost::bind
	     (&Dynamic::computeBodyPosition, this, currentJoint, _1, _2),
	     0,
	     "sotDynamicPr2(" + getName () + ")::output(matrix)::" + signame);
	    genericSignalRefs_.push_back (sig);
	    signalRegistration (*sig);
	  }
    }

    MatrixHomogeneous&
    Dynamic::computeBodyPosition (CjrlJoint* joint,
				  MatrixHomogeneous& position,
				  int t)
    {
      // get the position of joint from jrl-dyn and put it into
      // position.
	  for(uint i=0; i<position.nbRows(); ++i)
	    for(uint j=0; j<position.nbCols(); ++j)
		  position.elementAt(i,j) = joint->initialPosition().m[i*4+j];
      return position;
    }

    DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(Dynamic, "DynamicPr2");
  } // end of namespace pr2.
} // end of namespace sot.
