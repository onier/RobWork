
/**
 * We need to specify the wrapper classes,
 */
#include "LuaMath.hpp"
#include <rw/sensor.hpp>

#ifndef RWS_LUA_RWSTUDIO_HPP
#define RWS_LUA_RWSTUDIO_HPP


namespace rws {
namespace lua {
namespace rwstudio {

    void openFile(const std::string& filename);

    rw::common::PropertyMap& getPropertyMap(){
        return _propMap;
    }

    void setWorkcell(rw::models::WorkCellPtr workcell);

    rw::proximity::CollisionDetector* getCollisionDetector() {
        return _detector.get();
    }

    rwlibs::drawable::WorkCellGLDrawer* getWorkCellGLDrawer() {
        return &_workcellGLDrawer;
    }

    const rw::trajectory::TimedStatePath& getTimedStatePath() {
        return _timedStatePath;
    }

    void setTimedStatePath(const rw::trajectory::TimedStatePath& path);

    void setState(const rw::kinematics::State& state);

    const rw::kinematics::State& getState() { return _state; }

    rw::common::Log& log();

    void saveViewGL(const QString& filename);

    void updateAndRepaint();

    ViewGL* getView();



}}}


#endif
