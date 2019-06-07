%module rw_task

%{
#include <rwlibs/swig/ScriptTypes.hpp>
#include <rw/common/Ptr.hpp>

using namespace rwlibs::swig;
using rwlibs::task::Task;
%}

%import <rwlibs/swig/rw.i>

template <class T>
class Task
{
};

%template (TaskSE3) Task<rw::math::Transform3D<double> >;
%template (TaskSE3Ptr) rw::common::Ptr<Task<rw::math::Transform3D<double> > >;
OWNEDPTR(Task<rw::math::Transform3D<double> > );

class GraspTask {
public:
    GraspTask():
    GraspTask(rw::common::Ptr<Task<rw::math::Transform3D<double> > > task);
    rw::common::Ptr<Task<rw::math::Transform3D<double> > > toCartesianTask();
    std::string getGripperID();
    std::string getTCPID();
    std::string getGraspControllerID();
    void setGripperID(const std::string& id);
    void setTCPID(const std::string& id);
    void setGraspControllerID(const std::string& id);
    static std::string toString(GraspResult::TestStatus status);
    static void saveUIBK(rw::common::Ptr<GraspTask> task, const std::string& name );
    static void saveRWTask(rw::common::Ptr<GraspTask> task, const std::string& name );
    static void saveText(rw::common::Ptr<GraspTask> task, const std::string& name );
    static rw::common::Ptr<GraspTask> load(const std::string& name);
    static rw::common::Ptr<GraspTask> load(std::istringstream& inputStream);
    rw::common::Ptr<GraspTask> clone();
};

%template (GraspTaskPtr) rw::common::Ptr<GraspTask>;
OWNEDPTR(GraspTask);

class GraspResult {
public:
	enum TestStatus {
        UnInitialized = 0,
        Success, // 1
        CollisionInitially, // 2
        ObjectMissed, // 3
        ObjectDropped, // 4
        ObjectSlipped, // 5
        TimeOut, // 6
        SimulationFailure, // 7
        InvKinFailure, // 8
        PoseEstimateFailure, // 9
        CollisionFiltered, // 10
        CollisionObjectInitially, // 11
        CollisionEnvironmentInitially, // 12
        CollisionDuringExecution, // 13
        Interference, // 14
        WrenchInsufficient, // 15
        SizeOfStatusArray
     };
};
%template (GraspResultPtr) rw::common::Ptr<GraspResult>;
OWNEDPTR(GraspResult);

/* @} */

//TODO add other grasptask related classes
