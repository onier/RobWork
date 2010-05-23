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


#ifndef RWLIBS_SIMULATION_CAMERA_FRAMEGRAPPER_HPP
#define RWLIBS_SIMULATION_CAMERA_FRAMEGRAPPER_HPP

/**
 * @file FrameGrabber.hpp
 */

#include <rw/sensor/Image25D.hpp>
#include <rw/kinematics/Frame.hpp>

namespace rwlibs { namespace simulation {
    /** @addtogroup simulation */
    /* @{ */

    /**
     * @brief The FrameGrabber25D abstract interface, can be used to grab images from a
     * specialized source.
     */
    class FrameGrabber25D
    {
    public:
        /**
         * @brief constructor
         * @param width [in] width of the image that this FrameGrabber25D uses.
         * @param height [in] height of the image that this FrameGrabber25D uses.
         * @param encoding [in] color encoding of the image that this FrameGrabber25D uses.
         */
        FrameGrabber25D(int width, int height)
        {
            _img = new rw::sensor::Image25D( width, height );
        }

        /**
         * @brief destructor
         */
        virtual ~FrameGrabber25D()
        {
            delete _img;
        }

        /**
         * @brief returns the width of the image
         * @return the height of the image
         */
        int getWidth() { return _img->getWidth(); }

        /**
         * @brief returns the height of the image
         * @return the height of the image
         */
        int getHeight() { return _img->getHeight(); }

        /**
         * @brief resizes the image that this frameGrabber use. The colorcode will
         * default to the one that FrameGrabber25D was initialized with.
         * @param width [in] width of image
         * @param height [in] height of image
         */
        void resize(int width, int height) {
            //delete _img;
            _img = new rw::sensor::Image25D(width, height);
        };

        /**
         * @brief returns the image
         * @return the image
         */
        virtual rw::sensor::Image25D& getImage()
        {
            return *_img;
        }

        /**
         * @brief this function grabs a image from the specialized source and
         * copies it to the FrameGrabber25D image.
         */
        virtual void grab(rw::kinematics::Frame *frame,
                          const rw::kinematics::State& state) = 0;

        virtual double getMaxDepth() = 0;
        virtual double getMinDepth() = 0;

    protected:
        //! @brief The image
        rw::sensor::Image25D *_img;

    };

    typedef rw::common::Ptr<FrameGrabber25D> FrameGrabber25DPtr;

    /* @} */
}} // end namespaces

#endif // end include guard
