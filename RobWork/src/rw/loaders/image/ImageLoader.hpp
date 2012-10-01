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


#ifndef RW_LOADERS_IMAGE_IMAGELOADER_HPP
#define RW_LOADERS_IMAGE_IMAGELOADER_HPP

#include <rw/sensor/Image.hpp>

namespace rw { namespace loaders {

    /** @addtogroup loaders */
    /*@{*/

    /**
     * @brief Image loader interface
     *
     */
	class ImageLoader
	{
	public:
	    typedef rw::common::Ptr<ImageLoader> Ptr;

        /**
         * @param filename [in] name of the file that is to be loaded.
         * @return if loaded successfully a pointer to the image is returned else NULL
         */
		virtual rw::sensor::Image::Ptr loadImage(const std::string& filename) = 0;

		/**
         * @param img [in] the image that is to be saved.
         * @param filename [in] name of the file where the image is to be saved.
         * @return if loaded successfully a pointer to the image is returned else NULL
         */
		//void save(rw::sensor::Image::Ptr img, const std::string& filename);

		/**
		 * @brief
		 */
/*
		class Plugin: public rw::plugin::PluginFactory<ImageLoader> {
		public:

		    std::vector<std::string> getSupportedExtensions();

		protected:
		    void addSupportedExtension(const std::string& ext);
		};
		*/


        /**
         * @ingroup extensionpoints
         *
         * @brief a factory for ImageLoader. This factory also defines an
         * extension point for image loaders.
         *
         *
         */
#ifdef skdnmfdslknf
        class Factory: public ExtensionPoint<ImageLoader> {
        public:
            Factory():ExtensionPoint<ImageLoader>("rw.loaders.ImageLoader", "Example extension point"){};

            /**
             * @brief get an image loader for a specific file format
             * @param
             * @return
             */
            static std::vector<rw::common::Ptr<Extension> > getImageLoader(const std::string& format);
            static std::vector<rw::common::Ptr<Extension> > hasImageLoader(const std::string& format);

            /**
             * @brief get a list of supported formats
             * @return
             */
            static std::vector<std::string> getSupportedFormats();
        };
#endif
	};
	/*@}*/
}}

#endif /*RW_LOADERS_PGMLOADER_HPP*/
