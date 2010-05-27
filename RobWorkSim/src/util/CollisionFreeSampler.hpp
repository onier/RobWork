/*
 * FiniteStateSampler.hpp
 *
 *  Created on: 23-09-2009
 *      Author: jimali
 */

#ifndef COLLISIONFREESAMPLER_HPP_
#define COLLISIONFREESAMPLER_HPP_

#include "StateSampler.hpp"

#include <rw/proximity/CollisionDetector.hpp>

/**
 * @brief samples another state sampler until it returns a collision free
 * state.
 */
class CollisionFreeSampler: public StateSampler
{
public:

    /**
     * @brief constructor
     * @param sampler [in] the sampler that is to be wrapped
     * @param detector [in] the collision detector
     * @param n [in] max nr of tries pr sample request
     */
    CollisionFreeSampler(StateSamplerPtr sampler, rw::proximity::CollisionDetectorPtr detector, int n=-1);

    /**
     * @brief destructor
     */
    virtual ~CollisionFreeSampler();

    //! @copydoc StateSampler::sample
    bool sample(rw::kinematics::State& state);

    //! @copydoc StateSampler::empty
    bool empty() const{ return _sampler->empty(); };

private:
    StateSamplerPtr _sampler;
    rw::proximity::CollisionDetectorPtr _detector;
    int _n;
};

#endif /* FINITESTATESAMPLER_HPP_ */
