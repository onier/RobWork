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

#ifndef RWLIBS_ALGORITHMS_CAMERAMARKERPOSESAMPLER_HPP_
#define RWLIBS_ALGORITHMS_CAMERAMARKERPOSESAMPLER_HPP_

#include <rw/sensor/CameraModel.hpp>
#include <rw/models/Device.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/trajectory/Trajectory.hpp>

namespace rwlibs {
namespace algorithms {

/**
 * @brief calculate the stable poses of an object lying on a planar support
 * structure.
 */
class CameraMarkerPoseSampler {

    //! default constructor
    CameraMarkerPoseSampler();

	//! constructor
    CameraMarkerPoseSampler(rw::models::WorkCell::Ptr wc, std::vector<rw::kinematics::Frame*> markers);

    void addMarker( rw::kinematics::Frame* markerFrame );

    /**
     * @brief add a camera to the scene
     * @param cmodel
     */
    void addCamera( rw::sensor::CameraModel::Ptr cmodel);

    /**
     * @brief add a robot
     * @param dev
     */
    void addRobot( rw::models::Device::Ptr dev );

    void excludeMarker( const std::string& markerID);
    void excludeCamera( const std::string& cameraID);
    void excludeRobot( const std::string& robotID);

    /**
     * @brief set the volume in which the position part of samples of the marker pose is allowed.
     * @param volume [in] a geometric primitive
     * @param offset [in] the offset of the volume
     * @param refframe [in] name of refference frame. If empty WORLD frame is used
     */
    void includeVolume( const rw::geometry::Primitive& volume,
                          const rw::math::Transform3D<>& offset,
                          const std::string& refframe );

    ////// control of planning behavior

    /**
     * @brief force a certain percentage of samples to be visible in multiple views.
     * If zero no samples are forced to be in multiple views. If >0 then the planner will
     * strive to generate the specified amount of multiview samples.
     * @param percent [in] the percentage of samples that should be visible in multiple cameras.
     */
    void setForceMultiViewSamples(double percent);

    /**
     * @brief set the default camera uncertainty. The uncertainty describe a box around the current
     * camera position in which the user belive the camera to be in. The uncertainty essentially widens the
     * camera view frustrum to such as to make sure to sample the edges of the frustrum.
     * @param posUncertainty [in] length in m
     * @param angleUncertainty [in] angle in degree
     */
    void setCameraPoseUncertainty(double posUncertainty, double angleUncertainty);

    /**
     * @brief same as setCameraPoseUncertainty(double , double ) but for a single camera.
     * @param cam_name [in] name of camera
     * @param posUncertainty [in] length in m
     * @param angleUncertainty [in] angle in degree
     */
    void setCameraPoseUncertainty(const std::string& cam_name, double posUncertainty, double angleUncertainty);

    /**
     * @brief performs the offline work in generating samples in the workspace.
     * this is allways called automatically or manually before planning trajectories
     * to targets.
     */
    void generateSamples();

    /**
     * @brief plan the path
     * @param from
     * @return
     */
    rw::trajectory::StatePath::Ptr planNextTrajectory(const rw::kinematics::State& from);

    // getter functions for information on calculated temporary data

    /**
     * @brief test if the pose is inside the workspace of the robot
     * @param wTmpose [in]
     * @return true if wTmpose is inside workspace, false otherwise
     */
    bool isInsideWorkspace(const rw::math::Transform3D<>& wTmpose);

    /**
     * @brief test if the pose wTmpose, is visible to the camera, or more
     * specifically the camera frustrum.
     * @param wTmpose [in] pose to test
     * @param cam [in] camera with model parameters
     * @param state [in] the state of the system.
     * @return true if wTmpose is inside camera frustrum, false otherwise.
     */
    bool isInsideCameraFrustrum(const rw::math::Transform3D<>& wTmpose,
                                    rw::sensor::CameraModel::Ptr cam,
                                    rw::kinematics::State& state);

private:
    void addMultiSample(rw::sensor::CameraModel::Ptr cam1,
                           rw::sensor::CameraModel::Ptr cam2,
                           const rw::math::Transform3D<>& mpose);

private:
    std::vector<rw::sensor::CameraModel::Ptr> _camModels;
    std::map<rw::sensor::CameraModel*, bool> _getCamStatus;
    std::map<rw::sensor::CameraModel*, std::vector<rw::math::Transform3D<> > > _camPoses;
    std::map<
        std::pair<rw::sensor::CameraModel*,rw::sensor::CameraModel*>,
        std::vector<rw::math::Transform3D<> > > _camPosesMulti;
    rw::models::WorkCell::Ptr _wc;
    std::vector<rw::geometry::Primitive > _allowedWorkspace;
};

}
}

#endif /* STABLEPLANEPOSE_HPP_ */
