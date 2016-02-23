/********************************************************************************
 * Copyright 2013 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#ifndef RW_LOADERS_SAVERASSIMP_HPP_
#define RW_LOADERS_SAVERASSIMP_HPP_

/**
 * @file SaverAssimp.hpp
 *
 * \copydoc rw::loaders::SaverAssimp
 */

#include <rw/graphics/Model3D.hpp>
#include <string>

namespace rw {
namespace loaders {
//! @addtogroup loaders

//! @{
/**
 * @brief Save 3D models using the Open Asset Import Library (Assimp).
 *
 * For further information on Assimp and supported formats, see http://assimp.sourceforge.net
 *
 * So far the saver has been tested for .dae files (Collada) and .stl files.
 *
 * Note that the RobWork saver for Assimp is still work in progress.
 */
class SaverAssimp {
public:
	/**
	 * @brief Save a Model3D to a file. The format is inferred from the filename. If no valid extension found, Collada .dae file is created.
	 * @param model [in] the model to save.
	 * @param filename [in] the filename to save to - the extension determines the format.
	 */
	static void save(rw::graphics::Model3D::Ptr model, const std::string& filename);

private:
	//! @brief Constructor
	SaverAssimp();

	//! @brief Destructor
	virtual ~SaverAssimp();
};
//! @}
} /* namespace loaders */
} /* namespace rw */
#endif /* RW_LOADERS_SAVERASSIMP_HPP_ */
