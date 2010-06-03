/*
 * TactileMultiAxisSimSensor.hpp
 *
 *  Created on: 25-08-2008
 *      Author: jimali
 */

#include "TactileMultiAxisSimSensor.hpp"

#include <rw/kinematics/Kinematics.hpp>

using namespace rw::sensor;
using namespace rw::kinematics;

TactileMultiAxisSimSensor::TactileMultiAxisSimSensor(const std::string& name, dynamics::Body *body):
    TactileMultiAxisSensor(name, "TactileMultiAxisSensor")
{
	this->attachTo( &body->getBodyFrame() );
}

rw::math::Transform3D<> TactileMultiAxisSimSensor::getTransform(){
	return _transform;
}

rw::math::Vector3D<> TactileMultiAxisSimSensor::getForce(){

}

rw::math::Vector3D<> TactileMultiAxisSimSensor::getTorque(){

}

void TactileMultiAxisSimSensor::addForceW(const rw::math::Vector3D<>& point,
               const rw::math::Vector3D<>& force,
               const rw::math::Vector3D<>& cnormal,
               dynamics::Body *body)
{
	addForce(_fTw*point, _fTw.R()*force, _fTw.R()*cnormal, body);
}

void TactileMultiAxisSimSensor::addForce(const rw::math::Vector3D<>& point,
              const rw::math::Vector3D<>& force,
              const rw::math::Vector3D<>& cnormal,
              dynamics::Body *body)
{

}

void TactileMultiAxisSimSensor::update(double dt, rw::kinematics::State& state){
	// update aux variables
	_wTf = Kinematics::worldTframe( getFrame(), state);
	_fTw = inverse(_wTf);
}

void TactileMultiAxisSimSensor::reset(const rw::kinematics::State& state){
	_wTf = Kinematics::worldTframe( getFrame(), state);
	_fTw = inverse(_wTf);
}


