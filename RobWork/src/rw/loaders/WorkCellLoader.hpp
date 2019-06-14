/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute, 
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


#ifndef RW_LOADERS_WORKCELLLOADER_HPP
#define RW_LOADERS_WORKCELLLOADER_HPP

/**
 * @file WorkCellLoader.hpp
 */

#include <string>
#include <rw/graphics/WorkCellScene.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/common/ExtensionPoint.hpp>

// Forward declarations
//namespace rw { namespace models { class WorkCell; }}

namespace rw { namespace loaders {

    /** @addtogroup loaders */
    /* @{*/

    /**
     * @brief Extendible interface for loading of WorkCells from files.
     *
     * By default, the following formats are supported:
     *
     * - File extensions ".wu", ".wc", ".tag", ".dev" will be loaded using
     *   the TULLoader.
     * - Remaining file extensions will be loaded using the standard %RobWork
     *   %XML format (XMLRWLoader).
     *
     * The Factory defines an extension point "rw.loaders.WorkCellLoader"
     * that makes it possible to add loaders for other file formats than the
     * ones above. Extensions take precedence over the default loaders.
     *
     * The WorkCell loader is chosen based on a case-insensitive file extension
     * name. So "scene.wc.xml" will be loaded by the same loader as
     * "scene.WC.XML"
     *
     * WorkCells are supposed to be loaded using the Factory::load function:
     * \code{.cpp}
     * WorkCell::Ptr wc = WorkCellLoader::Factory::load("scene.wc.xml");
     * if (wc.isNull())
     *     RW_TRHOW("WorkCell could not be loaded.");
     * \endcode
     *
     * Alternatively a WorkCell can be loaded in the less convenient way:
     * \code{.cpp}
     * WorkCellLoader::Ptr loader = WorkCellLoader::Factory::getWorkCellLoader(".wc.xml");
     * WorkCell::Ptr wc = loader.loadWorkCell("scene.wc.xml");
     * if (wc.isNull())
     *     RW_TRHOW("WorkCell could not be loaded.");
     * \endcode
     */
    class WorkCellLoader
    {
    public:
        //! @brief Smart pointer of WorkCellLoader.
        typedef rw::common::Ptr<WorkCellLoader> Ptr;

        //! @brief Destructor.
        virtual ~WorkCellLoader(){}

        /**
         * @brief Load a WorkCell from a file.
         * @param filename [in] path to workcell file.
         */
        virtual models::WorkCell::Ptr loadWorkCell(const std::string& filename) = 0;

    	/**
    	 * @addtogroup extensionpoints
    	 * @extensionpoint{rw::loaders::WorkCellLoader::Factory, rw::loaders::WorkCellLoader, rw.loaders.WorkCellLoader}
 	 	 */


        /**
         * @brief A factory for WorkCellLoader. This factory also defines the
         * "rw.loaders.WorkCellLoader" extension point where new loaders can be
         * registered.
         */
        class Factory: public rw::common::ExtensionPoint<WorkCellLoader> {
            public:
                /**
                 * @brief Get loaders for a specific format.
                 * @param format [in] the extension (including initial dot).
                 * The extension name is case-insensitive.
                 * @return a suitable loader.
                 */
                static rw::common::Ptr<WorkCellLoader> getWorkCellLoader(
                        const std::string& format);

                /**
                 * @brief Loads/imports a WorkCell from a file.
                 *
                 * An exception is thrown if the file can't be loaded.
                 * The %RobWork %XML format is supported by default, as well as
                 * TUL WorkCell format.
                 * @param filename [in] name of the WorkCell file.
                 */
                static models::WorkCell::Ptr load(const std::string& filename);

            private:
                Factory(): rw::common::ExtensionPoint<WorkCellLoader>(
                        "rw.loaders.WorkCellLoader",
                        "Extension point for for WorkCell loaders.")
                {}
        };

    protected:
	    //! @brief Constructor.
		WorkCellLoader() {}
    };

	/**
	 * @brief Shortcut type for the WorkCellLoader::Factory
	 * @deprecated Please use WorkCellLoader::Factory instead.
	 */
    typedef WorkCellLoader::Factory WorkCellFactory;
    /**@}*/
}} // end namespaces

#endif // end include guard
