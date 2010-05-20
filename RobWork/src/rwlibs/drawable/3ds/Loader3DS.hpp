#ifndef RW_DRAWABLE_LOADER_3DS_H
#define RW_DRAWABLE_LOADER_3DS_H

#include <rwlibs/drawable/Model3D.hpp>
#include <rwlibs/drawable/Model3DLoader.hpp>

#include <cstdio>
#include <cstring>
#include <vector>
#include <string>

namespace rwlibs {
namespace drawable {


	class Loader3DS: public Model3DLoader
	{
	public:
		Loader3DS(){};
		virtual ~Loader3DS(){};

		Model3DPtr load(const std::string& name); // Loads a model

	private:
		//FILE *_bin3ds;			// The binary 3ds file
		//Model3D *_model;

		std::string _path; // The path of the model

	};
}
}
#endif // RW_DRAWABLE_LOADER_3DS_H
