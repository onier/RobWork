/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RW_LOADERS_DOM_DOMPROPERTYMAPSAVER_HPP_
#define RW_LOADERS_DOM_DOMPROPERTYMAPSAVER_HPP_

/**
 * @file DOMPropertyMapSaver.hpp
 *
 * \copydoc rw::loaders::DOMPropertyMapSaver
 */

#include <rw/common/Ptr.hpp>

#include <string>

namespace rw { namespace common { class DOMElem; } }
namespace rw { namespace common { class PropertyMap; } }
namespace rw { namespace common { class PropertyBase; } }

namespace rw {
namespace loaders {
//! @addtogroup loaders

//! @{
/**
 * @brief Class for saving rw::common::PropertyMap to XML.
 *
 * The saver is capable of saving all types defined in rw::common::PropertyType.
 *
 * Implemented using RobWork DOM saver abstraction.
 */
class DOMPropertyMapSaver {
public:
    /**
     * @brief Saves a Property to DOMElement.
     * @throws rw::common::Exception if an error occurs.
     * @param property [in] the Property to save.
     * @param element [in] DOMElement describing Property.
     */
	static void saveProperty(rw::common::Ptr<const rw::common::PropertyBase> property, rw::common::DOMElem& element);

    /**
     * @brief Saves a PropertyMap to DOMElement.
     * @throws rw::common::Exception if an error occurs.
     * @param map [in] the PropertyMap to save.
     * @param element [in/out] DOMElement describing PropertyMap.
     */
    static void saveProperties(const rw::common::PropertyMap& map, rw::common::DOMElem& element);

    /**
     * @brief Save rw::common::PropertyMap to file.
     * @throws rw::common::Exception if an error occurs.
     * @param map [in] the PropertyMap to save.
     * @param filename [in] file to save to.
     * @param schemaFileName [in] (optional) schema to use.
     */
    static void save(const rw::common::PropertyMap& map, const std::string& filename, const std::string& schemaFileName = "");

    /**
     * @brief Write rw::common::PropertyMap to ostream
     * @throws rw::common::Exception if an error occurs.
     * @param map [in] the PropertyMap to save.
     * @param ostream [in/out] output stream to write to.
     * @param schemaFileName [in] (optional) schema to use.
     */
    static void write(const rw::common::PropertyMap& map, std::ostream& outstream, const std::string& schemaFileName = "");

private:
    DOMPropertyMapSaver();
    virtual ~DOMPropertyMapSaver();
};
//! @}
} /* namespace loaders */
} /* namespace rw */
#endif /* RW_LOADERS_DOM_DOMPROPERTYMAPSAVER_HPP_ */
