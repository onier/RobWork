/*
 * ODEBody.hpp
 *
 *  Created on: 09-09-2008
 *      Author: jimali
 */

#ifndef ODEJOINT_HPP_
#define ODEJOINT_HPP_

#include <ode/ode.h>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/kinematics/State.hpp>
#include <dynamics/Body.hpp>
#include <dynamics/RigidJoint.hpp>
#include <rw/math/Vector3D.hpp>

/**
 * @brief this class bridges ODE's joints with RobWork joints.
 */
class ODEJoint {
public:
    typedef enum{FIXED, RIGID, DEPEND} ODEJointType;
    typedef enum{Revolute, Prismatic} JointType;

    /**
     * @brief constructor
     * @param odeJoint
     * @param odeMotor
     * @param rwbody
     * @return
     */
    ODEJoint(JointType jtype,
             dJointID odeJoint,
             dJointID odeMotor,
             dBodyID body,
             dynamics::RigidJoint* rwbody);

    ODEJoint(JointType jtype,
    		 dJointID odeJoint,
             dJointID odeMotor,
             dBodyID body,
             ODEJoint* owner,
             rw::kinematics::Frame *bframe,
             double scale, double off);

    virtual ~ODEJoint(){};

    void setVelocity(double vel){
        if(_jtype==Revolute) dJointSetAMotorParam(_motorId, dParamVel, vel);
        else dJointSetLMotorParam(_motorId, dParamVel, vel);
    }

    double getVelocity(){
        double vel;
        if(_jtype==Revolute) vel = dJointGetAMotorParam(_motorId, dParamVel);
        else vel = dJointGetLMotorParam(_motorId, dParamVel);
    	return vel;
    }

    void setAngle(double pos){
    	if(_jtype==Revolute) dJointSetAMotorAngle(_motorId, 0, pos);
    	//else dJointSetLMotorAngle(_motorId, 0, pos);
    }

    double getAngle(){
    	double val;
    	if(_jtype==Revolute) val = dJointGetHingeAngle( _jointId );
    	else val = dJointGetSliderPosition ( _jointId );
    	return val;
    }

    double getActualVelocity(){
    	if(_jtype==Revolute)  return dJointGetHingeAngleRate( _jointId );
    	return dJointGetSliderPositionRate ( _jointId );
    }

    void setMaxForce(double force){
    	if(_jtype==Revolute) dJointSetAMotorParam(_motorId,dParamFMax, force );
    	else dJointSetLMotorParam(_motorId,dParamFMax, force );
    }

    double getMaxForce(){
    	if(_jtype==Revolute)return dJointGetAMotorParam(_motorId,dParamFMax);
    	return dJointGetLMotorParam(_motorId,dParamFMax);
    }

    ODEJoint* getOwner(){
        return _owner;
    }

    dBodyID getODEBody(){
        return _bodyId;
    }

    ODEJointType getType(){
        return _type;
    }

    void reset(const rw::kinematics::State& state);

    double getScale(){ return _scale; };
    double getOffset(){ return _off; };

    //static ODEJoint* make(RevoluteJoint* joint, dBodyID parent);

private:
    dBodyID _bodyId;
    dJointID _jointId, _motorId;
    dynamics::RigidJoint *_rwJoint;

    ODEJoint *_owner;
    double _scale,_off;
    ODEJointType _type;
    JointType _jtype;

    rw::kinematics::Frame *_bodyFrame;
    rw::math::Vector3D<> _offset;


};


#endif /* ODEBODY_HPP_ */
