
/**
 * We need to specify the wrapper classes,
 */
//#include "LuaMath.hpp"
#include <rw/sensor.hpp>

#ifndef RWS_LUA_RWSTUDIO_HPP
#define RWS_LUA_RWSTUDIO_HPP

#include <rws/RobWorkStudio.hpp>
#include <rw/trajectory.hpp>
#include <rw/trajectory/Path.hpp>
#include <rw/kinematics.hpp>
#include <rwlibs/lua/LuaRobWork.hpp>

namespace rws {
namespace lua {
namespace rwstudio {

class RobWorkStudio {
public:
	RobWorkStudio(rws::RobWorkStudio* rws);

	void openFile(const std::string& filename);

	rw::common::PropertyMap& getPropertyMap();

	void setWorkcell(rw::models::WorkCellPtr workcell);

	rw::proximity::CollisionDetector* getCollisionDetector();

	rwlibs::drawable::WorkCellGLDrawer* getWorkCellGLDrawer();

	const rwlibs::lua::trajectory::TimedStatePath getTimedStatePath();

	void setTimedStatePath(const rwlibs::lua::trajectory::TimedStatePath& path);

	void setState(const rwlibs::lua::kinematics::State& state);

	const rwlibs::lua::kinematics::State getState();

	rw::common::Log& log();

	void saveViewGL(const std::string& filename);

	void updateAndRepaint();

	rws::ViewGL* getView();

	rws::RobWorkStudio *_rws;
};

RobWorkStudio* getRobWorkStudio();

	void setRobWorkStudio(rws::RobWorkStudio* rwstudio);

}}}


#endif
