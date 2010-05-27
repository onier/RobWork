#include "SpherePoseSampler.hpp"

using namespace rw::kinematics;
using namespace rw::math;

SpherePoseSampler::SpherePoseSampler(MovableFrame* mframe,
                                     const Vector3D<>& wPc, State& initState) :
    _mframe(mframe),
    _wPc(wPc),
    _initState(initState)
{
}

SpherePoseSampler::~SpherePoseSampler()
{
}

void SpherePoseSampler::setRPYNoiseBound(const rw::math::RPY<>& low, const rw::math::RPY<>& upper){

}

void SpherePoseSampler::setPosNoiseBound(const rw::math::Vector3D<>& low, const rw::math::Vector3D<>& upper){

}


bool SpherePoseSampler::sample(State& state)
{
    return true;
}


bool SpherePoseSampler::empty() const{ return false; };
