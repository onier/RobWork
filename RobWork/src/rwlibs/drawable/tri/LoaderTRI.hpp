#ifndef RW_DRAWABLE_LOADERTRI_H
#define RW_DRAWABLE_LOADERTRI_H

#include <rwlibs/drawable/Model3D.hpp>
#include <rwlibs/drawable/Model3DLoader.hpp>

#include <cstdio>
#include <cstring>
#include <vector>
#include <string>

namespace rwlibs {
namespace drawable {


	class LoaderTRI: public Model3DLoader
	{
	public:
		LoaderTRI();
		virtual ~LoaderTRI();

		Model3DPtr load(const std::string& name); // Loads a model

	};
}
}
#endif // RW_DRAWABLE_LOADER_3DS_H
