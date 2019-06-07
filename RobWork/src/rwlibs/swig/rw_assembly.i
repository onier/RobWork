%module rw_assembly

%{
#include <rwlibs/swig/ScriptTypes.hpp>
#include <rw/common/Ptr.hpp>

using namespace rwlibs::swig;
using rw::trajectory::Path;
using rw::trajectory::Timed;
using rw::trajectory::Trajectory;
using rwlibs::task::Task;
%}

%include <std_string.i>
%include <std_vector.i>

%import <rwlibs/swig/rw.i>
%import <rwlibs/swig/rw_task.i>

%pragma(java) jniclassimports=%{
import org.robwork.rw.*;
import org.robwork.rw_task.*;
%}
%typemap(javaimports) SWIGTYPE %{
import org.robwork.rw.*;
import org.robwork.rw_task.*;
%}

class AssemblyControlResponse
{
public:
	AssemblyControlResponse();
	virtual ~AssemblyControlResponse();
	
	/*
	typedef enum Type {
		POSITION,    //!< Position control
		VELOCITY,    //!< Velocity control
		HYBRID_FT_POS//!< Hybrid position and force/torque control
	} Type;

	Type type;
	*/
	
	rw::math::Transform3D<double>  femaleTmaleTarget;
	rw::common::Ptr<Trajectory<rw::math::Transform3D<double> > > worldTendTrajectory;
	rw::math::VelocityScrew6D<double>  femaleTmaleVelocityTarget;
	rw::math::Rotation3D<double>  offset;
	//VectorND<6,bool> selection;
	rw::math::Wrench6D<double> force_torque;
	bool done;
	bool success;
};

%template (AssemblyControlResponsePtr) rw::common::Ptr<AssemblyControlResponse>;
OWNEDPTR(AssemblyControlResponse);

/**
 * @brief Interface for assembly control strategies.
 */
class AssemblyControlStrategy
{
public:
	AssemblyControlStrategy();
	virtual ~AssemblyControlStrategy();
	
	/*
	class ControlState {
	public:
		//! @brief smart pointer type to this class
	    typedef rw::common::Ptr<ControlState> Ptr;

		//! @brief Constructor.
		ControlState() {};

		//! @brief Destructor.
		virtual ~ControlState() {};
	};
	virtual rw::common::Ptr<ControlState> createState() const;
	*/
	
	//virtual rw::common::Ptr<AssemblyControlResponse> update(rw::common::Ptr<AssemblyParameterization> parameters, rw::common::Ptr<AssemblyState> real, rw::common::Ptr<AssemblyState> assumed, rw::common::Ptr<ControlState> controlState, State &state, FTSensor* ftSensor, double time) const = 0;
	virtual rw::math::Transform3D<double>  getApproach(rw::common::Ptr<AssemblyParameterization> parameters) = 0;
	virtual std::string getID() = 0;
	virtual std::string getDescription() = 0;
	virtual rw::common::Ptr<AssemblyParameterization> createParameterization(const rw::common::Ptr<PropertyMap> map) = 0;
};

%template (AssemblyControlStrategyPtr) rw::common::Ptr<AssemblyControlStrategy>;

class AssemblyParameterization
{
public:
	AssemblyParameterization();
	AssemblyParameterization(rw::common::Ptr<PropertyMap> pmap);
	virtual ~AssemblyParameterization();
	virtual rw::common::Ptr<PropertyMap> toPropertyMap() const;
	virtual rw::common::Ptr<AssemblyParameterization> clone() const;
};

%template (AssemblyParameterizationPtr) rw::common::Ptr<AssemblyParameterization>;
OWNEDPTR(AssemblyParameterization);

class AssemblyRegistry
{
public:
	AssemblyRegistry();
	virtual ~AssemblyRegistry();
	void addStrategy(const std::string id, rw::common::Ptr<AssemblyControlStrategy> strategy);
	std::vector<std::string> getStrategies() const;
	bool hasStrategy(const std::string& id) const;
	rw::common::Ptr<AssemblyControlStrategy> getStrategy(const std::string &id) const;
};

%template (AssemblyRegistryPtr) rw::common::Ptr<AssemblyRegistry>;
OWNEDPTR(AssemblyRegistry);

class AssemblyResult
{
public:
	AssemblyResult();
	AssemblyResult(rw::common::Ptr<Task<rw::math::Transform3D<double> > > task);
	virtual ~AssemblyResult();
	rw::common::Ptr<AssemblyResult> clone() const;
	rw::common::Ptr<Task<rw::math::Transform3D<double> > > toCartesianTask();
	static void saveRWResult(rw::common::Ptr<AssemblyResult> result, const std::string& name);
	static void saveRWResult(std::vector<rw::common::Ptr<AssemblyResult> > results, const std::string& name);
	static std::vector<rw::common::Ptr<AssemblyResult> > load(const std::string& name);
	static std::vector<rw::common::Ptr<AssemblyResult> > load(std::istringstream& inputStream);
	
	bool success;
	//Error error;
	rw::math::Transform3D<double>  femaleTmaleEnd;

	std::string taskID;
	std::string resultID;
    
    Path<Timed<AssemblyState> > realState;
	Path<Timed<AssemblyState> > assumedState;
	rw::math::Transform3D<double>  approach;
	std::string errorMessage;
};

%template (AssemblyResultPtr) rw::common::Ptr<AssemblyResult>;
%template (AssemblyResultPtrVector) std::vector<rw::common::Ptr<AssemblyResult> >;
OWNEDPTR(AssemblyResult);

class AssemblyState
{
public:
	AssemblyState();
	//AssemblyState(rw::common::Ptr<Target<rw::math::Transform3D<double> > > target);
	virtual ~AssemblyState();
	//static rw::common::Ptr<Target<rw::math::Transform3D<double> > > toCartesianTarget(const AssemblyState &state);

	std::string phase;
	rw::math::Transform3D<double>  femaleOffset;
	rw::math::Transform3D<double>  maleOffset;
	rw::math::Transform3D<double>  femaleTmale;
	rw::math::Wrench6D<double> ftSensorMale;
	rw::math::Wrench6D<double> ftSensorFemale;
	bool contact;
	Path<rw::math::Transform3D<double> > maleflexT;
	Path<rw::math::Transform3D<double> > femaleflexT;
	Path<rw::math::Transform3D<double> > contacts;
	rw::math::Vector3D<double> maxContactForce;
};

%template (AssemblyStatePtr) rw::common::Ptr<AssemblyState>;
%template (TimedAssemblyState) Timed<AssemblyState>;
%template (TimedAssemblyStateVector) std::vector<Timed<AssemblyState> >;
%template (PathTimedAssemblyState) Path<Timed<AssemblyState> >;
OWNEDPTR(AssemblyState);

class AssemblyTask
{
public:
	AssemblyTask();
	AssemblyTask(rw::common::Ptr<Task<rw::math::Transform3D<double> > > task, rw::common::Ptr<AssemblyRegistry> registry = NULL);
	virtual ~AssemblyTask();
	rw::common::Ptr<Task<rw::math::Transform3D<double> > > toCartesianTask();
	static void saveRWTask(rw::common::Ptr<AssemblyTask> task, const std::string& name);
	static void saveRWTask(std::vector<rw::common::Ptr<AssemblyTask> > tasks, const std::string& name);
	static std::vector<rw::common::Ptr<AssemblyTask> > load(const std::string& name, rw::common::Ptr<AssemblyRegistry> registry = NULL);
	static std::vector<rw::common::Ptr<AssemblyTask> > load(std::istringstream& inputStream, rw::common::Ptr<AssemblyRegistry> registry = NULL);
	rw::common::Ptr<AssemblyTask> clone() const;

	std::string maleID;
    std::string femaleID;
    rw::math::Transform3D<double>  femaleTmaleTarget;
    rw::common::Ptr<AssemblyControlStrategy> strategy;
    rw::common::Ptr<AssemblyParameterization> parameters;
    
    std::string maleTCP;
    std::string femaleTCP;
    
    std::string taskID;
    std::string workcellName;
    std::string generator;
    std::string date;
    std::string author;
    
    std::string malePoseController;
    std::string femalePoseController;
    std::string maleFTSensor;
    std::string femaleFTSensor;
    std::vector<std::string> maleFlexFrames;
    std::vector<std::string> femaleFlexFrames;
    std::vector<std::string> bodyContactSensors;
};

%template (AssemblyTaskPtr) rw::common::Ptr<AssemblyTask>;
%template (AssemblyTaskPtrVector) std::vector<rw::common::Ptr<AssemblyTask> >;
OWNEDPTR(AssemblyTask);