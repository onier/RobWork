
#include "Loader3DS.hpp"
#include <rwlibs/os/rwgl.hpp>
#include <cmath>                       // Header file for the math library
#include <iostream>

#include "Model3DS.hpp"
#include <rw/common/macros.hpp>
#include <rw/common/StringUtil.hpp>
using namespace rw::common;
using namespace rwlibs::drawable;

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

Model3DPtr Loader3DS::load(const std::string& name)
{

	Model3DS model3ds;
	setlocale(LC_ALL, "C");
	model3ds.Load(name);
	setlocale(LC_ALL, "");

	// TODO: convert 3ds model into rw model


	Model3DPtr model;
	return model;
}
