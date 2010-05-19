/*
 * Model3DLoader.hpp
 *
 *  Created on: 28-03-2010
 *      Author: jimali
 */

#ifndef MODEL3DLOADER_HPP_
#define MODEL3DLOADER_HPP_

#include "Model3D.hpp"

namespace rwlibs {
namespace drawable {
	/**
	 * @brief interface for classes that are able to load 3d models
	 */
    class Model3DLoader {
    public:

        virtual Model3DPtr load(const std::string& filename) = 0;

        //virtual void save(Model3DPtr model, const std::string& filename) = 0;

    };

}
}
#endif /* MODEL3DLOADER_HPP_ */
