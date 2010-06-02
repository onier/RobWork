#include "LuaRWStudio.hpp"

#include <rw/common.hpp>

using namespace rws::lua::rwstudio;
#include <iostream>
using namespace std;
#include <sstream>

#define NS rw::math

namespace
{
    string eToString(const rw::common::Exception& e)
    {
        ostringstream buf;
        buf << e.getMessage();
        return buf.str();
    }

    template <typename T>
    string toString(const T& x)
    {
        ostringstream buf;
        buf << x;
        return buf.str();
    }
}

RobWorkStudio::RobWorkStudio(rws::RobWorkStudio* rws):_rws(rws)
{}

void RobWorkStudio::openFile(const std::string& filename){
	_rws->openFile(filename);
}

rw::common::PropertyMap& RobWorkStudio::getPropertyMap(){
	return _rws->getPropertyMap();
}

void RobWorkStudio::setWorkcell(rw::models::WorkCellPtr workcell){
	_rws->setWorkcell( workcell );
}

rw::proximity::CollisionDetector* RobWorkStudio::getCollisionDetector(){
	return _rws->getCollisionDetector();
}

rwlibs::drawable::WorkCellGLDrawer* RobWorkStudio::getWorkCellGLDrawer(){
	return _rws->getWorkCellGLDrawer();
}

const rwlibs::lua::trajectory::TimedStatePath RobWorkStudio::getTimedStatePath(){
	return _rws->getTimedStatePath();
}

void RobWorkStudio::setTimedStatePath(const rwlibs::lua::trajectory::TimedStatePath& path){
	_rws->setTimedStatePath(path);
}

void RobWorkStudio::setState(const rwlibs::lua::kinematics::State& state){
	_rws->setState(state);
}

const rwlibs::lua::kinematics::State RobWorkStudio::getState(){
	return _rws->getState();
}

rw::common::Log& RobWorkStudio::log(){
	return _rws->log();
}

void RobWorkStudio::saveViewGL(const std::string& filename){
	_rws->saveViewGL(QString( filename.c_str() ));
}

void RobWorkStudio::updateAndRepaint(){
	_rws->updateAndRepaint();
}

rws::ViewGL* RobWorkStudio::getView(){
	return _rws->getView();
}

RobWorkStudio *rwstudio_internal = NULL;

RobWorkStudio* rws::lua::rwstudio::getRobWorkStudio(){
	return rwstudio_internal;
}

void rws::lua::rwstudio::setRobWorkStudio(rws::RobWorkStudio* rwstudio){
	rwstudio_internal = new RobWorkStudio(rwstudio);
}

