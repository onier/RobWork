/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#ifndef RWLIBS_SIMULATION_CAMERA_GLFRAMEGRAPPER_HPP
#define RWLIBS_SIMULATION_CAMERA_GLFRAMEGRAPPER_HPP

/**
 * @file GLFrameGrabber.hpp
 */

#include "FrameGrabber.hpp"

#include <rwlibs/drawable/WorkCellGLDrawer.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/PerspectiveTransform3D.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/State.hpp>

namespace rwlibs { namespace simulation {
    /** @addtogroup simulation */
    /* @{ */

    /**
     * @brief An implementation of the FrameGrabber interface. The GLFrameGrabber
     * grabs images from a OpenGL scene using a simple pinhole camera model.
     *
     * a framethe opengl rendering to
     * take pictures of the scene.

     * The most basic parameter of a camera is its Field of view. This
     * can be used as an initial camera model. Field of view can be
     * calculated from the focal length and the size of the CCD
     * typically (1/2, 1/3, 1/4) inch.
     * If a more realistic camera model is
     * required the perspective transform of a specific camera can be added
     */
    class GLFrameGrabber : public FrameGrabber
    {
    public:
        /**
         * @brief constructor
         * @param width [in] width of image
         * @param height [in] height of image
         * @param fov [in] the vertical field of view angle in degree
         * @param drawer [in] the WorkCellGLDrawer that draws the OpenGL scene
         * @param state [in] the state of the workcell
         */
        GLFrameGrabber(
            int width, int height, double fov,
            rwlibs::drawable::WorkCellGLDrawer *drawer,
            rw::kinematics::State &state)
            :
            FrameGrabber(width,height,rw::sensor::Image::RGB),
            _fieldOfView(fov),_drawer(drawer),
            _perspTrans(rw::math::Transform3D<double>::identity())
        {}

        /**
         * @brief destructor
         */
        virtual ~GLFrameGrabber(){};

        /**
         * @copydoc FrameGrabber::grab
         */
        void grab(rw::kinematics::Frame* frame, const rw::kinematics::State& state);

    private:
        double _fieldOfView; // in the y-axis
        rwlibs::drawable::WorkCellGLDrawer *_drawer;
        rw::math::Transform3D<double> _perspTrans;
    };

    /* @} */
}} // end namespaces

#endif // end include guard
