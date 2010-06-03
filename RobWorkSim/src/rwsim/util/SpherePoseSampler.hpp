/*
 * FiniteStateSampler.hpp
 *
 *  Created on: 23-09-2009
 *      Author: jimali
 */

#ifndef SPHEREPOSESAMPLER_HPP_
#define SPHEREPOSESAMPLER_HPP_

#include "StateSampler.hpp"

#include <rw/math/Vector3D.hpp>
#include <rw/kinematics/MovableFrame.hpp>

/**
 * @brief samples poses of a movable frame, such that the frame is always
 * positioned on a sphere around some specified center. Random deviations
 * to the position of the frame can be added.
 *
 * This StateSampler will never become empty.
 *
 *
 *
 */
class SpherePoseSampler: public StateSampler
{
public:

    /**
     * @brief constructor \b frame will be set in random poses on
     * the sphere with center wPc and radius \b radi
     * @param mframe [in] the frame
     * @param wPc [in] the origo of the sphere
     * @param radi [in] radius of sphere
     */
    SpherePoseSampler(rw::kinematics::MovableFrame* mframe,
                      const rw::math::Vector3D<>& wPc,
                      double radi);

    /**
     * @brief constructor \b frame will be set in random poses on
     * the sphere with center wPc and radius implicitly defined as
     * the distance from \b frame to
     * \b wPc in \b initState
     * @param mframe [in] the frame
     * @param wPc [in] the origo of the sphere
     * @param initState [in] the initial state
     */
    SpherePoseSampler(rw::kinematics::MovableFrame* mframe,
                      const rw::math::Vector3D<>& wPc,
                      rw::kinematics::State& initState);

    /**
     * @brief destructor
     */
    virtual ~SpherePoseSampler();

    /**
     * @brief sets the bounds of the rotation noise(small random deviation)
     * that is added to the generated pose of the frame.
     * @param low [in] lower bound RPY values
     * @param upper [in] upper bound RPY values
     */
    void setRPYNoiseBound(const rw::math::RPY<>& low, const rw::math::RPY<>& upper);

    /**
     * @brief sets the bounds of the position noise(small random deviation)
     * that is added to the generated pose of the frame.
     * @param low [in] lower bound position values
     * @param upper [in] upper bound position values
     */
    void setPosNoiseBound(const rw::math::Vector3D<>& low, const rw::math::Vector3D<>& upper);

    //! @copydoc StateSampler::sample
    bool sample(rw::kinematics::State& state);

    //! @copydoc StateSampler::empty
    bool empty() const;

private:
    rw::kinematics::MovableFrame* _mframe;
    rw::math::Vector3D<> _wPc;
    rw::kinematics::State _initState;
};

#endif /* FINITESTATESAMPLER_HPP_ */
