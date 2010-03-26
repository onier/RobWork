/*
** Lua binding: LuaRWStudio
** Generated automatically by tolua++-1.0.92 on 03/11/10 00:58:24.
*/

#ifndef __cplusplus
#include "stdlib.h"
#endif
#include "string.h"

#include "tolua++.h"

/* Exported function */
TOLUA_API int  tolua_LuaRWStudio_open (lua_State* tolua_S);

#include <rw/math.hpp>
#include "LuaRWStudio.hpp"

/* function to release collected object via destructor */
#ifdef __cplusplus

static int tolua_collect_rws__lua__rwstudio__RobWorkStudio (lua_State* tolua_S)
{
 rws::lua::rwstudio::RobWorkStudio* self = (rws::lua::rwstudio::RobWorkStudio*) tolua_tousertype(tolua_S,1,0);
	delete self;
	return 0;
}
#endif


/* function to register type */
static void tolua_reg_types (lua_State* tolua_S)
{
 tolua_usertype(tolua_S,"rwlibs::lua::kinematics::State");
 tolua_usertype(tolua_S,"rwlibs::lua::trajectory::TimedStatePath");
 tolua_usertype(tolua_S,"rws::lua::rwstudio::RobWorkStudio");
 tolua_usertype(tolua_S,"rw::common::PropertyMap");
 tolua_usertype(tolua_S,"rwlibs::drawable::WorkCellGLDrawer");
 tolua_usertype(tolua_S,"rw::common::Log");
 tolua_usertype(tolua_S,"rw::models::WorkCellPtr");
 tolua_usertype(tolua_S,"rw::proximity::CollisionDetector");
 tolua_usertype(tolua_S,"rws::ViewGL");
 tolua_usertype(tolua_S,"rws::RobWorkStudio");
}

/* method: new of class  rws::lua::rwstudio::RobWorkStudio */
#ifndef TOLUA_DISABLE_tolua_LuaRWStudio_rws_lua_rwstudio_RobWorkStudio_new00
static int tolua_LuaRWStudio_rws_lua_rwstudio_RobWorkStudio_new00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"rws::lua::rwstudio::RobWorkStudio",0,&tolua_err) ||
     !tolua_isusertype(tolua_S,2,"rws::RobWorkStudio",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  rws::RobWorkStudio* rws = ((rws::RobWorkStudio*)  tolua_tousertype(tolua_S,2,0));
  {
   rws::lua::rwstudio::RobWorkStudio* tolua_ret = (rws::lua::rwstudio::RobWorkStudio*)  new rws::lua::rwstudio::RobWorkStudio(rws);
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"rws::lua::rwstudio::RobWorkStudio");
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'new'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: new_local of class  rws::lua::rwstudio::RobWorkStudio */
#ifndef TOLUA_DISABLE_tolua_LuaRWStudio_rws_lua_rwstudio_RobWorkStudio_new00_local
static int tolua_LuaRWStudio_rws_lua_rwstudio_RobWorkStudio_new00_local(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"rws::lua::rwstudio::RobWorkStudio",0,&tolua_err) ||
     !tolua_isusertype(tolua_S,2,"rws::RobWorkStudio",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  rws::RobWorkStudio* rws = ((rws::RobWorkStudio*)  tolua_tousertype(tolua_S,2,0));
  {
   rws::lua::rwstudio::RobWorkStudio* tolua_ret = (rws::lua::rwstudio::RobWorkStudio*)  new rws::lua::rwstudio::RobWorkStudio(rws);
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"rws::lua::rwstudio::RobWorkStudio");
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'new'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: openFile of class  rws::lua::rwstudio::RobWorkStudio */
#ifndef TOLUA_DISABLE_tolua_LuaRWStudio_rws_lua_rwstudio_RobWorkStudio_openFile00
static int tolua_LuaRWStudio_rws_lua_rwstudio_RobWorkStudio_openFile00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"rws::lua::rwstudio::RobWorkStudio",0,&tolua_err) ||
     !tolua_iscppstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  rws::lua::rwstudio::RobWorkStudio* self = (rws::lua::rwstudio::RobWorkStudio*)  tolua_tousertype(tolua_S,1,0);
  const std::string filename = ((const std::string)  tolua_tocppstring(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'openFile'",NULL);
#endif
  {
   self->openFile(filename);
   tolua_pushcppstring(tolua_S,(const char*)filename);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'openFile'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: getPropertyMap of class  rws::lua::rwstudio::RobWorkStudio */
#ifndef TOLUA_DISABLE_tolua_LuaRWStudio_rws_lua_rwstudio_RobWorkStudio_getPropertyMap00
static int tolua_LuaRWStudio_rws_lua_rwstudio_RobWorkStudio_getPropertyMap00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"rws::lua::rwstudio::RobWorkStudio",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  rws::lua::rwstudio::RobWorkStudio* self = (rws::lua::rwstudio::RobWorkStudio*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'getPropertyMap'",NULL);
#endif
  {
   rw::common::PropertyMap& tolua_ret = (rw::common::PropertyMap&)  self->getPropertyMap();
   tolua_pushusertype(tolua_S,(void*)&tolua_ret,"rw::common::PropertyMap");
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'getPropertyMap'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: setWorkcell of class  rws::lua::rwstudio::RobWorkStudio */
#ifndef TOLUA_DISABLE_tolua_LuaRWStudio_rws_lua_rwstudio_RobWorkStudio_setWorkcell00
static int tolua_LuaRWStudio_rws_lua_rwstudio_RobWorkStudio_setWorkcell00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"rws::lua::rwstudio::RobWorkStudio",0,&tolua_err) ||
     !tolua_isusertype(tolua_S,2,"rw::models::WorkCellPtr",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  rws::lua::rwstudio::RobWorkStudio* self = (rws::lua::rwstudio::RobWorkStudio*)  tolua_tousertype(tolua_S,1,0);
  rw::models::WorkCellPtr workcell = *((rw::models::WorkCellPtr*)  tolua_tousertype(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'setWorkcell'",NULL);
#endif
  {
   self->setWorkcell(workcell);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'setWorkcell'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: getCollisionDetector of class  rws::lua::rwstudio::RobWorkStudio */
#ifndef TOLUA_DISABLE_tolua_LuaRWStudio_rws_lua_rwstudio_RobWorkStudio_getCollisionDetector00
static int tolua_LuaRWStudio_rws_lua_rwstudio_RobWorkStudio_getCollisionDetector00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"rws::lua::rwstudio::RobWorkStudio",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  rws::lua::rwstudio::RobWorkStudio* self = (rws::lua::rwstudio::RobWorkStudio*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'getCollisionDetector'",NULL);
#endif
  {
   rw::proximity::CollisionDetector* tolua_ret = (rw::proximity::CollisionDetector*)  self->getCollisionDetector();
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"rw::proximity::CollisionDetector");
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'getCollisionDetector'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: getWorkCellGLDrawer of class  rws::lua::rwstudio::RobWorkStudio */
#ifndef TOLUA_DISABLE_tolua_LuaRWStudio_rws_lua_rwstudio_RobWorkStudio_getWorkCellGLDrawer00
static int tolua_LuaRWStudio_rws_lua_rwstudio_RobWorkStudio_getWorkCellGLDrawer00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"rws::lua::rwstudio::RobWorkStudio",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  rws::lua::rwstudio::RobWorkStudio* self = (rws::lua::rwstudio::RobWorkStudio*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'getWorkCellGLDrawer'",NULL);
#endif
  {
   rwlibs::drawable::WorkCellGLDrawer* tolua_ret = (rwlibs::drawable::WorkCellGLDrawer*)  self->getWorkCellGLDrawer();
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"rwlibs::drawable::WorkCellGLDrawer");
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'getWorkCellGLDrawer'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: getTimedStatePath of class  rws::lua::rwstudio::RobWorkStudio */
#ifndef TOLUA_DISABLE_tolua_LuaRWStudio_rws_lua_rwstudio_RobWorkStudio_getTimedStatePath00
static int tolua_LuaRWStudio_rws_lua_rwstudio_RobWorkStudio_getTimedStatePath00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"rws::lua::rwstudio::RobWorkStudio",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  rws::lua::rwstudio::RobWorkStudio* self = (rws::lua::rwstudio::RobWorkStudio*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'getTimedStatePath'",NULL);
#endif
  {
   const rwlibs::lua::trajectory::TimedStatePath& tolua_ret = (const rwlibs::lua::trajectory::TimedStatePath&)  self->getTimedStatePath();
   tolua_pushusertype(tolua_S,(void*)&tolua_ret,"const rwlibs::lua::trajectory::TimedStatePath");
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'getTimedStatePath'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: setTimedStatePath of class  rws::lua::rwstudio::RobWorkStudio */
#ifndef TOLUA_DISABLE_tolua_LuaRWStudio_rws_lua_rwstudio_RobWorkStudio_setTimedStatePath00
static int tolua_LuaRWStudio_rws_lua_rwstudio_RobWorkStudio_setTimedStatePath00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"rws::lua::rwstudio::RobWorkStudio",0,&tolua_err) ||
     !tolua_isusertype(tolua_S,2,"const rwlibs::lua::trajectory::TimedStatePath",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  rws::lua::rwstudio::RobWorkStudio* self = (rws::lua::rwstudio::RobWorkStudio*)  tolua_tousertype(tolua_S,1,0);
  const rwlibs::lua::trajectory::TimedStatePath* path = ((const rwlibs::lua::trajectory::TimedStatePath*)  tolua_tousertype(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'setTimedStatePath'",NULL);
#endif
  {
   self->setTimedStatePath(*path);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'setTimedStatePath'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: setState of class  rws::lua::rwstudio::RobWorkStudio */
#ifndef TOLUA_DISABLE_tolua_LuaRWStudio_rws_lua_rwstudio_RobWorkStudio_setState00
static int tolua_LuaRWStudio_rws_lua_rwstudio_RobWorkStudio_setState00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"rws::lua::rwstudio::RobWorkStudio",0,&tolua_err) ||
     !tolua_isusertype(tolua_S,2,"const rwlibs::lua::kinematics::State",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  rws::lua::rwstudio::RobWorkStudio* self = (rws::lua::rwstudio::RobWorkStudio*)  tolua_tousertype(tolua_S,1,0);
  const rwlibs::lua::kinematics::State* state = ((const rwlibs::lua::kinematics::State*)  tolua_tousertype(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'setState'",NULL);
#endif
  {
   self->setState(*state);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'setState'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: getState of class  rws::lua::rwstudio::RobWorkStudio */
#ifndef TOLUA_DISABLE_tolua_LuaRWStudio_rws_lua_rwstudio_RobWorkStudio_getState00
static int tolua_LuaRWStudio_rws_lua_rwstudio_RobWorkStudio_getState00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"rws::lua::rwstudio::RobWorkStudio",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  rws::lua::rwstudio::RobWorkStudio* self = (rws::lua::rwstudio::RobWorkStudio*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'getState'",NULL);
#endif
  {
   const rwlibs::lua::kinematics::State& tolua_ret = (const rwlibs::lua::kinematics::State&)  self->getState();
   tolua_pushusertype(tolua_S,(void*)&tolua_ret,"const rwlibs::lua::kinematics::State");
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'getState'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: log of class  rws::lua::rwstudio::RobWorkStudio */
#ifndef TOLUA_DISABLE_tolua_LuaRWStudio_rws_lua_rwstudio_RobWorkStudio_log00
static int tolua_LuaRWStudio_rws_lua_rwstudio_RobWorkStudio_log00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"rws::lua::rwstudio::RobWorkStudio",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  rws::lua::rwstudio::RobWorkStudio* self = (rws::lua::rwstudio::RobWorkStudio*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'log'",NULL);
#endif
  {
   rw::common::Log& tolua_ret = (rw::common::Log&)  self->log();
   tolua_pushusertype(tolua_S,(void*)&tolua_ret,"rw::common::Log");
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'log'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: saveViewGL of class  rws::lua::rwstudio::RobWorkStudio */
#ifndef TOLUA_DISABLE_tolua_LuaRWStudio_rws_lua_rwstudio_RobWorkStudio_saveViewGL00
static int tolua_LuaRWStudio_rws_lua_rwstudio_RobWorkStudio_saveViewGL00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"rws::lua::rwstudio::RobWorkStudio",0,&tolua_err) ||
     !tolua_iscppstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  rws::lua::rwstudio::RobWorkStudio* self = (rws::lua::rwstudio::RobWorkStudio*)  tolua_tousertype(tolua_S,1,0);
  const std::string filename = ((const std::string)  tolua_tocppstring(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'saveViewGL'",NULL);
#endif
  {
   self->saveViewGL(filename);
   tolua_pushcppstring(tolua_S,(const char*)filename);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'saveViewGL'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: updateAndRepaint of class  rws::lua::rwstudio::RobWorkStudio */
#ifndef TOLUA_DISABLE_tolua_LuaRWStudio_rws_lua_rwstudio_RobWorkStudio_updateAndRepaint00
static int tolua_LuaRWStudio_rws_lua_rwstudio_RobWorkStudio_updateAndRepaint00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"rws::lua::rwstudio::RobWorkStudio",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  rws::lua::rwstudio::RobWorkStudio* self = (rws::lua::rwstudio::RobWorkStudio*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'updateAndRepaint'",NULL);
#endif
  {
   self->updateAndRepaint();
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'updateAndRepaint'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: getView of class  rws::lua::rwstudio::RobWorkStudio */
#ifndef TOLUA_DISABLE_tolua_LuaRWStudio_rws_lua_rwstudio_RobWorkStudio_getView00
static int tolua_LuaRWStudio_rws_lua_rwstudio_RobWorkStudio_getView00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"rws::lua::rwstudio::RobWorkStudio",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  rws::lua::rwstudio::RobWorkStudio* self = (rws::lua::rwstudio::RobWorkStudio*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'getView'",NULL);
#endif
  {
   rws::ViewGL* tolua_ret = (rws::ViewGL*)  self->getView();
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"rws::ViewGL");
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'getView'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: rws::lua::rwstudio::getRobWorkStudio */
#ifndef TOLUA_DISABLE_tolua_LuaRWStudio_rws_lua_rwstudio_getRobWorkStudio00
static int tolua_LuaRWStudio_rws_lua_rwstudio_getRobWorkStudio00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnoobj(tolua_S,1,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  {
   rws::lua::rwstudio::RobWorkStudio* tolua_ret = (rws::lua::rwstudio::RobWorkStudio*)  rws::lua::rwstudio::getRobWorkStudio();
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"rws::lua::rwstudio::RobWorkStudio");
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'getRobWorkStudio'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* Open function */
TOLUA_API int tolua_LuaRWStudio_open (lua_State* tolua_S)
{
 tolua_open(tolua_S);
 tolua_reg_types(tolua_S);
 tolua_module(tolua_S,NULL,0);
 tolua_beginmodule(tolua_S,NULL);
  tolua_module(tolua_S,"rwlibs",0);
  tolua_beginmodule(tolua_S,"rwlibs");
   tolua_module(tolua_S,"lua",0);
   tolua_beginmodule(tolua_S,"lua");
    tolua_module(tolua_S,"trajectory",0);
    tolua_beginmodule(tolua_S,"trajectory");
     tolua_cclass(tolua_S,"TimedStatePath","rwlibs::lua::trajectory::TimedStatePath","",NULL);
     tolua_beginmodule(tolua_S,"TimedStatePath");
     tolua_endmodule(tolua_S);
    tolua_endmodule(tolua_S);
   tolua_endmodule(tolua_S);
  tolua_endmodule(tolua_S);
  tolua_module(tolua_S,"rws",0);
  tolua_beginmodule(tolua_S,"rws");
   tolua_module(tolua_S,"lua",0);
   tolua_beginmodule(tolua_S,"lua");
    tolua_module(tolua_S,"rwstudio",0);
    tolua_beginmodule(tolua_S,"rwstudio");
     #ifdef __cplusplus
     tolua_cclass(tolua_S,"RobWorkStudio","rws::lua::rwstudio::RobWorkStudio","",tolua_collect_rws__lua__rwstudio__RobWorkStudio);
     #else
     tolua_cclass(tolua_S,"RobWorkStudio","rws::lua::rwstudio::RobWorkStudio","",NULL);
     #endif
     tolua_beginmodule(tolua_S,"RobWorkStudio");
      tolua_function(tolua_S,"new",tolua_LuaRWStudio_rws_lua_rwstudio_RobWorkStudio_new00);
      tolua_function(tolua_S,"new_local",tolua_LuaRWStudio_rws_lua_rwstudio_RobWorkStudio_new00_local);
      tolua_function(tolua_S,".call",tolua_LuaRWStudio_rws_lua_rwstudio_RobWorkStudio_new00_local);
      tolua_function(tolua_S,"openFile",tolua_LuaRWStudio_rws_lua_rwstudio_RobWorkStudio_openFile00);
      tolua_function(tolua_S,"getPropertyMap",tolua_LuaRWStudio_rws_lua_rwstudio_RobWorkStudio_getPropertyMap00);
      tolua_function(tolua_S,"setWorkcell",tolua_LuaRWStudio_rws_lua_rwstudio_RobWorkStudio_setWorkcell00);
      tolua_function(tolua_S,"getCollisionDetector",tolua_LuaRWStudio_rws_lua_rwstudio_RobWorkStudio_getCollisionDetector00);
      tolua_function(tolua_S,"getWorkCellGLDrawer",tolua_LuaRWStudio_rws_lua_rwstudio_RobWorkStudio_getWorkCellGLDrawer00);
      tolua_function(tolua_S,"getTimedStatePath",tolua_LuaRWStudio_rws_lua_rwstudio_RobWorkStudio_getTimedStatePath00);
      tolua_function(tolua_S,"setTimedStatePath",tolua_LuaRWStudio_rws_lua_rwstudio_RobWorkStudio_setTimedStatePath00);
      tolua_function(tolua_S,"setState",tolua_LuaRWStudio_rws_lua_rwstudio_RobWorkStudio_setState00);
      tolua_function(tolua_S,"getState",tolua_LuaRWStudio_rws_lua_rwstudio_RobWorkStudio_getState00);
      tolua_function(tolua_S,"log",tolua_LuaRWStudio_rws_lua_rwstudio_RobWorkStudio_log00);
      tolua_function(tolua_S,"saveViewGL",tolua_LuaRWStudio_rws_lua_rwstudio_RobWorkStudio_saveViewGL00);
      tolua_function(tolua_S,"updateAndRepaint",tolua_LuaRWStudio_rws_lua_rwstudio_RobWorkStudio_updateAndRepaint00);
      tolua_function(tolua_S,"getView",tolua_LuaRWStudio_rws_lua_rwstudio_RobWorkStudio_getView00);
     tolua_endmodule(tolua_S);
     tolua_function(tolua_S,"getRobWorkStudio",tolua_LuaRWStudio_rws_lua_rwstudio_getRobWorkStudio00);
    tolua_endmodule(tolua_S);
   tolua_endmodule(tolua_S);
  tolua_endmodule(tolua_S);
 tolua_endmodule(tolua_S);
 return 1;
}


#if defined(LUA_VERSION_NUM) && LUA_VERSION_NUM >= 501
 TOLUA_API int luaopen_LuaRWStudio (lua_State* tolua_S) {
 return tolua_LuaRWStudio_open(tolua_S);
};
#endif

