#include "CameraMarkerPoseSampler.hpp"

#include <rw/math.hpp>

using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::sensor;
using namespace rwlibs::algorithms;


    //! default constructor
CameraMarkerPoseSampler::CameraMarkerPoseSampler()
{

}

    //! constructor
CameraMarkerPoseSampler::CameraMarkerPoseSampler(
        rw::models::WorkCell::Ptr wc, std::vector<rw::kinematics::Frame*> markers)
{

}

void CameraMarkerPoseSampler::addMarker( rw::kinematics::Frame* markerFrame )
{
}

void CameraMarkerPoseSampler::addCamera( rw::sensor::CameraModel::Ptr cmodel)
{

}

void CameraMarkerPoseSampler::addRobot( rw::models::Device::Ptr dev )
{

}

void CameraMarkerPoseSampler::excludeMarker( const std::string& )
{

}

void CameraMarkerPoseSampler::excludeCamera( const std::string& )
{

}

void CameraMarkerPoseSampler::excludeRobot( const std::string& )
{

}

void CameraMarkerPoseSampler::includeVolume( const rw::geometry::Primitive& volume,
                          const rw::math::Transform3D<>& offset,
                          const std::string& refframe )
{

}


void CameraMarkerPoseSampler::setForceMultiViewSamples(double percent)
{

}

void CameraMarkerPoseSampler::setCameraPoseUncertainty(double posUncertainty, double angleUncertainty)
{

}

void CameraMarkerPoseSampler::setCameraPoseUncertainty(
        const std::string& cam_name, double posUncertainty, double angleUncertainty)
{

}

namespace {


    void sampleCameraVolume(CameraModel::Ptr cam, std::vector<Transform3D<> >& samples, int nrSamples)
    {
        // samples are generated in camera frame
        Frame *rframe = cam->getFrame();
        // camera looks in the negative z direction
        rw::math::Vector3D<> dir = -Vector3D<>::z();
        // field of view
        double fovx = cam->getFieldOfViewX();
        double fovy = cam->getFieldOfViewY();
        // clipping planes
        double zFar = cam->getFarClippingPlane();
        double zNear = cam->getNearClippingPlane();

        double zNearPow3 = std::pow(zNear, 3);
        double zFarPow3 = std::pow(zFar, 3);


        for(int i=0; i<nrSamples; i++){
            // we make a rotation that has a random direction within the view frustrum
            rw::math::RPY<> ranRPY(0, rw::math::Math::ran(-fovx,fovx)*Deg2Rad, rw::math::Math::ran(-fovy,fovy)*Deg2Rad);
            rw::math::Rotation3D<> dirRot = ranRPY.toRotation3D();

            // the depth is sampled from parabolic distribution as the area of cross sections of the frustum is a quadratic
            // function of the depth plane's depth ( (z*2*tan(FOV/2))^2 * aspectr )

            double u = Math::ran();
            double z = std::pow( u * (zFarPow3 - zNearPow3) + zNearPow3, 0.33333333);
            rw::math::Vector3D<> point = (dirRot*dir) * z;

            // finally, we generate an orientation of the marker such that its z-axis is pointing
            // in the direction of the camera.
            //Vector3D<> mdir = - normalize(point);

            double maxMarkerCamAngle = 70*Deg2Rad;
            rw::math::RPY<> markRPY(0,
                                    rw::math::Math::ran(-maxMarkerCamAngle,maxMarkerCamAngle),
                                    rw::math::Math::ran(-maxMarkerCamAngle,maxMarkerCamAngle));

            rw::math::Rotation3D<> mdirRan = dirRot*markRPY.toRotation3D() * RPY<>(0,Pi,0).toRotation3D();

            samples.push_back( Transform3D<>(point, mdirRan ) );
        }
    }

}


void CameraMarkerPoseSampler::generateSamples()
{
    // here we randomly generate possible marker poses in the allowed volume of the workspace.
    // Inside camera views, visible to one or multiple cameras, inside volume constraints,
    State state = _wc->getDefaultState();


    std::vector<Transform3D<> > samples;
    // for each camera we densely sample their frustrum
    BOOST_FOREACH(CameraModel::Ptr cam, _camModels){
        samples.clear();
        sampleCameraVolume( cam, samples, 1000 );
        Transform3D<> wTcam = cam->getFrame()->wTf(state);
        // now filter all poses such that all are
        BOOST_FOREACH(Transform3D<>& sample, samples){
            // first transform sample to world
            Transform3D<> wTsample = wTcam * sample;
            // test if the sample is inside the allowed workspace
            if( !isInsideWorkspace(wTsample) )
                continue;

            bool multiSample = false;
            // next test if multiple cameras can see the marker
            BOOST_FOREACH(CameraModel::Ptr camera, _camModels ){
                if( camera == cam )
                    continue;

                if( isInsideCameraFrustrum(wTsample, camera, state) ) {
                    multiSample = true;
                    // add pose to multi camera poses map
                    addMultiSample(cam, camera, wTsample);
                }
            }


        }
    }

    // for each stereo camera we densely sample their frustrum
    //BOOST_FOREACH(CameraModel::Ptr cam, _camModels){
    //    samples.clear();
    //    sampleCameraVolume( cam, samples, 1000 );
    //}


/*
    _firstCall = false;
    q = robot_->randomQ();
    rw::kinematics::State state = robot_->getState(q);

    _iksolver   = new rw::invkin::JacobianIKSolver(robot_->getDevice(), state);
    _iksolver->setCheckJointLimits(true);
    _iksolver->setClampToBounds(true);
    double minDist = 0.2;
    // create a list of valid poses
    CoViSPointer<ch::CameraSystem> simulationCameraSystem_ = measurementSystem_->getSimulationCameraSystem();
    const int cameraCount = simulationCameraSystem_->getCameraCount();
    int acceptedViews = 0;



    for (int cameraNo = 0; cameraNo < cameraCount; cameraNo++) {

          rw::kinematics::Frame *refFrame = CoViSPointer<RobWorkCamera>::cast(simulationCameraSystem_->getCamera(cameraNo))->getReferenceFrame().get();
          rw::math::Transform3D<> zeroTcam = rw::kinematics::Kinematics::frameTframe(measurementSystem_->getReferenceFrame().get(), refFrame, state);

          rw::math::Transform3D<> wTcam = rw::kinematics::Kinematics::worldTframe(refFrame, state);
          rw::math::Vector3D<> dir = -Vector3D<>::z();
          // add 10000 points for each view
          // assume the camera is looking in
          for(int i=0; i<10000; i++){
              // firat randomly rotate vector
              rw::math::RPY<> ranRPY(0, rw::math::Math::ran(-25,25)*Deg2Rad, rw::math::Math::ran(-25,25)*Deg2Rad);
              rw::math::Rotation3D<> ranRot = ranRPY.toRotation3D();
              rw::math::Vector3D<> point = (ranRot*dir) * (minDist+rw::math::Math::ran(0,1.5));

              // transform point into reference system
              _pointsInView.push_back( zeroTcam * point );
          }
    }
    */


}

rw::trajectory::StatePath::Ptr CameraMarkerPoseSampler::planNextTrajectory(const rw::kinematics::State& from)
{
    return NULL;
}

bool CameraMarkerPoseSampler::isInsideWorkspace(const rw::math::Transform3D<>& mpose){
    // test if pose is inside allowed worlspace
    BOOST_FOREACH(rw::geometry::Primitive::Ptr& prim, _allowedWorkspace){
        prim->isConvex()
    }
}

void CameraMarkerPoseSampler::addMultiSample(rw::sensor::CameraModel::Ptr cam1,
                                             rw::sensor::CameraModel::Ptr cam2,
                                             const rw::math::Transform3D<>& mpose)
{
    std::pair<CameraModel*,CameraModel*> key(cam2.get(),cam1.get());
    if(cam1.get()<cam2.get())
        std::swap(key.first, key.second);
    _camPosesMulti[key].push_back( mpose );
}
