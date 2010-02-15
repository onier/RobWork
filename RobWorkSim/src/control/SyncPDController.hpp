#ifndef SYNCPDController_HPP_
#define SYNCPDController_HPP_

#include <rwlibs/control/JointController.hpp>
#include <rwlibs/control/SyncVelocityRamp.hpp>
#include <rwlibs/simulation/SimulatedController.hpp>

#include <dynamics/RigidDevice.hpp>

/**
 * @brief a JointController that use a PD loop on each joint
 * to control the velocity such that the position target is
 * reached at the same time. The PD controls the joint position and
 * velocity from a generated synchronous ramp profile.
 */
class SyncPDController: public JointController, public rwlibs::simulation::SimulatedController {

public:

    SyncPDController(RigidDevice* rdev, const rw::kinematics::State& state):
        JointController(&rdev->getModel()),
        _ddev(rdev),
        _time(0.0),
        _target(rdev->getModel().getQ(state)),
        _lastError(rw::math::Q::zero(rdev->getModel().getDOF())),
        _velramp(&(rdev->getModel())),
        _currentQ(_target)
    {
        _velramp.setTarget(_target,_target);
    }

    virtual ~SyncPDController(){};

    unsigned int getControlModes(){
        return POSITION || VELOCITY;
    }

    void setControlMode(ControlMode mode);

    void setTargetPos(const rw::math::Q& target);

    void setTargetVel(const rw::math::Q& vals);

    void setTargetAcc(const rw::math::Q& vals);

    /**
     * @brief updates the state of the dynamicdevice
     */
    void update(double dt, rw::kinematics::State& state);

    /**
     *
     * @param state
     */
    void reset(const rw::kinematics::State& state);

    rw::math::Q getQ(){
        return _currentQ;
    }

    rw::math::Q getQd(){ return _target;}

    Controller* getController(){ return this;};

private:
    RigidDevice *_ddev;
    double _time;
    rw::math::Q _target;
    rw::math::Q _lastError;
    rw::control::SyncVelocityRamp _velramp;
    rw::math::Q _currentQ;
    rw::math::Q _maxVel;
    rw::math::Q _x;
    int _mode;


};


#endif /*SyncPDController_HPP_*/
