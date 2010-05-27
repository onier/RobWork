/*
 * FiniteStateSampler.hpp
 *
 *  Created on: 23-09-2009
 *      Author: jimali
 */

#ifndef PRESHAPESAMPLER_HPP_
#define PRESHAPESAMPLER_HPP_

#include "StateSampler.hpp"

#include <rw/math/Vector3D.hpp>
#include <rw/models/Device.hpp>
#include <rw/pathplanning/QSampler.hpp>

/**
 * @brief
 *
 * This StateSampler will never become empty
 *
 */
class PreshapeSampler: public StateSampler
{
public:

    /**
     * @brief create a preshape sampler based on a QSampler
     * @param dev [in] the device for which configurations are sampled
     * @param qsampler [in] the configuration sampler
     * @param initState [in] the initial state
     */
    PreshapeSampler(rw::models::Device* dev,
    				rw::pathplanning::QSamplerPtr qsampler,
    				rw::kinematics::State& initState);

    /**
     * @brief destructor
     */
    virtual ~PreshapeSampler();



    //! @copydoc StateSampler::sample
    bool sample(rw::kinematics::State& state);

    //! @copydoc StateSampler::sample
    bool empty() const{ return false; };

private:
    rw::models::Device* _dev;
    rw::pathplanning::QSamplerPtr _qsampler;
    rw::kinematics::State _initState;
};

#endif /* FINITESTATESAMPLER_HPP_ */
