/*
 * FiniteStateSampler.hpp
 *
 *  Created on: 23-09-2009
 *      Author: jimali
 */

#ifndef FINITESTATESAMPLER_HPP_
#define FINITESTATESAMPLER_HPP_

#include "StateSampler.hpp"
#include <vector>


/**
 * @brief a state sampler that will sample a finite set of states. The nr
 * of samples is not necesdarilly finite since states are allowed to
 * be sampled multiple times.
 */
class FiniteStateSampler: public StateSampler
{
public:
	//! type of sampling
	typedef enum{
		ORDERED_SAMPLING, //! samples are choosen from front to back
		RANDOM_SAMPLING //! samples are choosen randomly
	} SamplerType;

    /**
     * @brief constructor for sampling a single state
     * @param state [in] the state to sample
     * @param n [in] number of allowed samples, if n<0 then a infinite
     * number of samples is allowed
     */
    FiniteStateSampler(const rw::kinematics::State& state, int n=1, SamplerType type = ORDERED_SAMPLING);

    /**
     * @brief constructor for sampling multiple states
     * @param states [in] states that are to be sampled
     * @param n [in] number of allowed samples, if n<0 then a infinite
     * number of samples is allowed
     */
    FiniteStateSampler(const std::vector<rw::kinematics::State>& states, int n=1, SamplerType type = ORDERED_SAMPLING);

    /**
     * @brief destructor
     */
    virtual ~FiniteStateSampler();

    /**
     * @brief add a single state to sample
     * @param state [in] state to sample
     */
    void addState(const rw::kinematics::State& state);

    /**
     * @brief add states to be sampled
     * @param states [in] states to sample
     */
    void addStates(const std::vector<rw::kinematics::State>& states);

    /**
     * @brief set the states that you want sampled
     * @param states [in] the states to sample
     */
    void setStates(const std::vector<rw::kinematics::State>& states);

    /**
     * @copydoc StateSampler::sample
     *
     * One of the states in the finite state list is choosen and returned
     */
    bool sample(rw::kinematics::State& state);

    //! @copydoc StateSampler::sample
    bool empty() const;

private:
	SamplerType _type;
	int _n,_cidx;
	std::vector<rw::kinematics::State> _states;
    bool _empty;


};

#endif /* FINITESTATESAMPLER_HPP_ */
