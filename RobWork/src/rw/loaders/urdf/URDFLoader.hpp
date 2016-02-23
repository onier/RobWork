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

#ifndef RW_LOADERS_URDFLOADER_HPP_
#define RW_LOADERS_URDFLOADER_HPP_

/**
 * @file URDFLoader.hpp
 *
 * \copydoc rw::loaders::URDFLoader
 */

#include <rw/common/DOMElem.hpp>
#include <rw/loaders/WorkCellLoader.hpp>

namespace rw {
namespace loaders {
//! @addtogroup loaders

//! @{
/**
 * @brief Load WorkCells that are specified in Unified Robot Description Format (URDF).
 *
 * For further information on the URDF format, see http://wiki.ros.org/urdf
 *
 * Main limitations of the format are that the format only represents a robot (and not a WorkCell as such).
 * A robot is a rigid tree structure connected by joints.
 *
 * Note that the RobWork loader for URDF files is work in progress. Only expect it to work for simple setups!
 */
class URDFLoader: public WorkCellLoader {
public:
	//! @brief Constructor
	URDFLoader();

	//! @brief Destructor
	virtual ~URDFLoader();

	//! @copydoc WorkCellLoader::loadWorkCell(const std::string&)
    rw::models::WorkCell::Ptr loadWorkCell(const std::string& filename);

    /**
     * @brief Load URDF file with given schema
     * @param filename [in] Filename of the file to load.
     * @param schemaFileName [in] Name of the schema to use. If empty it will use the schema specified in the XML-file if available.
     * @return a pointer to the WorkCell.
     */
    rw::models::WorkCell::Ptr loadWorkCell(const std::string& filename, const std::string& schemaFileName);

    /**
     * @brief Load URDF file from stream.
     * @param instream [in] The input stream to read from.
     * @param schemaFileName [in] Name of the schema to use. If empty it will use the schema specified in the XML-file if available.
     * @return a pointer to the WorkCell.
     */
    rw::models::WorkCell::Ptr loadWorkCell(std::istream& instream, const std::string& schemaFileName = "");

    /**
     * @brief Load WorkCell from DOMElem, assuming format is a valid URDF description.
     * @param element [in] DOMElement representing the WorkCell.
     * @return a pointer to the WorkCell.
     */
    rw::models::WorkCell::Ptr loadWorkCell(rw::common::DOMElem::Ptr element);

    //! @copydoc URDFLoader::loadWorkCell(const std::string&, const std::string&)
	static rw::models::WorkCell::Ptr load(const std::string& filename, const std::string& schemaFileName = "");

    //! @copydoc URDFLoader::loadWorkCell(std::istream&, const std::string&)
	static rw::models::WorkCell::Ptr load(std::istream& instream, const std::string& schemaFileName = "");

    //! @copydoc URDFLoader::loadWorkCell(rw::common::DOMElem::Ptr)
	static rw::models::WorkCell::Ptr load(rw::common::DOMElem::Ptr element);
};
//! @}
} /* namespace loaders */
} /* namespace rw */
#endif /* RW_LOADERS_URDFLOADER_HPP_ */
