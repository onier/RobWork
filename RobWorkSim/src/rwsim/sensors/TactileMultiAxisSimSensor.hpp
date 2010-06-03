/*
 * TactileMultiAxisSimSensor.hpp
 *
 *  Created on: 25-08-2008
 *      Author: jimali
 */

#ifndef TACTILEMULTIAXISSIMSENSOR_HPP_
#define TACTILEMULTIAXISSIMSENSOR_HPP_

#include "SimulatedTactileSensor.hpp"
#include <rw/sensor/TactileMultiAxisSensor.hpp>

/**
 * @brief A sensor that measures force and torque around some reference frame
 */
class TactileMultiAxisSimSensor: public rw::sensor::TactileMultiAxisSensor, public SimulatedTactileSensor {
public:

	/**
	 * @brief constructor
	 * @param name [in] identifier
	 * @param body [in] the body that this sensor is attached to
	 */
    TactileMultiAxisSimSensor(const std::string& name, dynamics::Body *body);

    /**
     * @brief destructor
     */
    virtual ~TactileMultiAxisSimSensor(){};

    //// Interface inherited from SimulatedSensor
    //! @copydoc SimulatedSensor::update
    void update(double dt, rw::kinematics::State& state);

    //! @copydoc SimulatedSensor::reset
    void reset(const rw::kinematics::State& state);

    //// Interface inherited from SimulatedTactileSensor
    //! @copydoc SimulatedTactileSensor::addForceW
    void addForceW(const rw::math::Vector3D<>& point,
                   const rw::math::Vector3D<>& force,
                   const rw::math::Vector3D<>& cnormal,
                   dynamics::Body *body = NULL);

    //! @copydoc SimulatedTactileSensor::addForce
    void addForce(const rw::math::Vector3D<>& point,
                  const rw::math::Vector3D<>& force,
                  const rw::math::Vector3D<>& cnormal,
                  dynamics::Body *body=NULL);


    //! @copydoc TactileMultiAxisSensor::getTransform
    rw::math::Transform3D<> getTransform();

    //!@copydoc TactileMultiAxisSensor::getForce
    rw::math::Vector3D<> getForce();


    //! @copydoc TactileMultiAxisSensor::getTorque
    rw::math::Vector3D<> getTorque();

    //! @copydoc TactileMultiAxisSensor::getMaxTorque
    double getMaxTorque(){return _maxTorque;};


    //! @copydoc TactileMultiAxisSensor::getMaxForce
    double getMaxForce(){return _maxForce;};

private:
    TactileMultiAxisSimSensor();

private:
    // the frame that the force and torque is described in relation to
    rw::math::Transform3D<> _transform;
    rw::math::Vector3D<> _force, _torque;
    double _maxForce,_maxTorque;

    //! aux variables updated through \b update
    rw::math::Transform3D<> _wTf, _fTw;
};


#endif /* TACTILEMULTIAXISSENSOR_HPP_ */
