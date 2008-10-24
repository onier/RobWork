/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.

 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/
#include "SimpleMultiSolver.hpp"

#include <rw/math/VelocityScrew6D.hpp>
#include <rw/math/LinearAlgebra.hpp>
#include <rw/math/Jacobian.hpp>
#include <rw/math/Quaternion.hpp>

#include <rw/common/Property.hpp>

#include <rw/models/SerialDevice.hpp>
#include <rw/models/TreeDevice.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/DeviceJacobian.hpp>
#include <rw/models/WorkCell.hpp>

#include <boost/shared_ptr.hpp>

using namespace boost;
using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::common;
using namespace rw::invkin;

using namespace boost::numeric;

namespace {

    std::vector<boost::shared_ptr<FKRange> > createFKRanges(const Frame *base,
                                                            const std::vector<Frame*>& foi,
                                                            const State& state)
    {
        std::vector<boost::shared_ptr<FKRange> > fkranges;
        for(size_t i = 0; i<foi.size(); i++){
            boost::shared_ptr<FKRange> range(new FKRange(base , foi[i], state) );
            fkranges.push_back( range );
        }
        return fkranges;
    }


}

SimpleMultiSolver::SimpleMultiSolver(const TreeDevice* device,
                                     const State& state):
    IterativeMultiIK(device->getEnds().size()),
    _device(device),
    _foi(device->getEnds()),
    _fkranges( createFKRanges(device->getBase(),_foi,state) ),
    _maxQuatStep(0.4),
    _returnBestFit(false)
{
    setMaxIterations(40);
    _jacCalc = device->baseDJframes( _foi, state );
}

SimpleMultiSolver::SimpleMultiSolver(const JointDevice* device,
                                     const std::vector<Frame*>& foi,
                                     const State& state):
     IterativeMultiIK(foi.size()),
     _device(device),
     _foi(foi),
     _fkranges(createFKRanges(device->getBase(),_foi,state)),
     _maxQuatStep(0.4),
     _returnBestFit(false)
 {
     setMaxIterations(40);
     _jacCalc = device->baseDJframes( foi, state );
 }

void SimpleMultiSolver::setMaxLocalStep(double quatlength, double poslength){
    _maxQuatStep = quatlength;
}

bool SimpleMultiSolver::solveLocal(
    const std::vector<Transform3D<> > &bTed,
    std::vector<double>& maxError,
    State &state,
    int maxIter,
    bool untilSmallChange) const
{
    //std::cout << "Perform local search!" << std::endl;

    //std::cout << bTed.size() << "==" << fkranges.size() << std::endl;
    RW_ASSERT(bTed.size()==_fkranges.size());
    //std::cout << "Create vector " << std::endl;
    boost::numeric::ublas::vector<double> b_eXed_vec(_fkranges.size()*6);
    double lastError = 0;
    Q q = _device->getQ(state);
    const int maxIterations = maxIter;
    for (int cnt = 0; cnt < maxIterations; ++cnt) {
        //std::cout << "Iteration: " << maxIterations << std::endl;
        //bool belowMaxError = true;
        for(size_t row=0;row<_fkranges.size(); row++){
            Transform3D<> bTe = _fkranges[row]->get(state);
            Transform3D<> eTed = inverse(bTe) * bTed[row];

            EAA<> e_eOed(eTed(2,1), eTed(0,2), eTed(1,0));
            Vector3D<> e_eVed = eTed.P();
            VelocityScrew6D<> e_eXed(e_eVed, e_eOed);
            VelocityScrew6D<> b_eXed = bTe.R() * e_eXed;

            // copy result to b_eXed_vector
            b_eXed_vec(row*6+0) = b_eXed(0);
            b_eXed_vec(row*6+1) = b_eXed(1);
            b_eXed_vec(row*6+2) = b_eXed(2);
            b_eXed_vec(row*6+3) = b_eXed(3);
            b_eXed_vec(row*6+4) = b_eXed(4);
            b_eXed_vec(row*6+5) = b_eXed(5);

            //if ( norm_inf(b_eXed) > maxError[row]*errorScale ) {
                //belowMaxError = false;
            //}
            //std::cout << b_eXed << std::endl;
        }
        double error = norm_2(b_eXed_vec);
        // std::cout << "Error: " << error << std::endl;
        if(!untilSmallChange){
            if( error  < maxError[0]/*belowMaxError*/ ){
                //std::cout << "below MAX Error: " << error << " "<<  b_eXed_vec << std::endl;
                return true;
            }
        } else {
            if( std::fabs(error-lastError)<maxError[0])
                return true;
            lastError = error;
        }

        Jacobian J = _jacCalc->get( state );
        const LinearAlgebra::Matrix& Jp =
            LinearAlgebra::pseudoInverse(J.m());

        Q dq ( prod( Jp , b_eXed_vec) );

        double dq_len = dq.normInf();
        if( dq_len > 0.8 )
            dq *= 0.8/dq_len;

        q += dq;

        _device->setQ(q, state);
    }
    return false;
}


#define USE_NEW_EXP
#ifdef USE_NEW_EXP
std::vector<Q> SimpleMultiSolver::solve(
    const std::vector<Transform3D<> >& bTeds,
    const State& initial_state) const

{
    int maxIterations = getMaxIterations();
    std::vector<double> maxError = getMaxError();
    std::vector<double> maxTmpError = maxError;
    State state = initial_state;

    const int N = bTeds.size();

    // for each end effector calculate the distance
    double length = 0;
    std::vector<Transform3D<> > bTes(N);
    std::vector<Vector3D<> > pDists(N);
    std::vector<Quaternion<> > q1s(N);
    std::vector<Quaternion<> > q2s(N);
    std::vector<Quaternion<> > qDists(N);
    for(int i=0; i<N; i++){
        bTes[i] = _fkranges[i]->get(state);
        pDists[i] = bTeds[i].P() - bTes[i].P();
        q1s[i] = Quaternion<>( bTes[i].R() );
        q2s[i] = Quaternion<>( bTeds[i].R() );
        qDists[i] = q2s[i]- q1s[i];
        maxTmpError[i] *= 1000;
        double len = qDists[i].getLength();
        if(len>length)
            length = len;
    }
    // now that we have the max length we can calculate the nr of steps

    const int steps = (int)ceil( length/_maxQuatStep );
    //std::cout << "Nr of Steps: " << steps << std::endl;
    // create a vector of the via targets
    std::vector<Transform3D<> > bTedVia(N);
    for(int step=1; step < steps; step++){
        // generate the via point target list from the interpolation
        double nStep = ((double)step) / (double)steps;
        for(int i=0; i<N; i++){
            //Quaternion<> qNext = q1s[i].slerp(q2s[i], nStep);
            Quaternion<> qNext = qDists[i];
            qNext *= nStep;
            qNext = q1s[i] + qNext;
            qNext.normalize();
            Vector3D<> pNext = bTes[i].P() + pDists[i]*nStep;
            bTedVia[i] = Transform3D<>(pNext,qNext);
        }
        // now perform newton iteration to the generated via point
        // we relax the error requirement a bit to alow for faster search
        solveLocal(bTedVia, maxTmpError, state, 5, true);
    }
    // now we perform yet another newton search with higher precision to determine
    // the end result
    std::vector<Q> result;
    if( solveLocal(bTeds, maxError, state, maxIterations, false) ){
        //std::cout << "result found!" << std::endl;
        result.push_back(_device->getQ(state));
    } else if( _returnBestFit ){
        result.push_back(_device->getQ(state));
    }
    return result;
}
#else
std::vector<Q> SimpleMultiSolver::solve(
    const std::vector<Transform3D<> >& bTed,
    const State& initial_state) const

{
    int maxIterations = getMaxIterations();
    std::vector<double> maxError = getMaxError();
    State state = initial_state;

    /* TODO: interpolation of bigger paths don't work...
    size_t n = bTed.size();
    // if the distance between current and end configuration is
    // too large then split it up in smaller steps

    std::vector<Quaternion<> > qDist( n );
    std::vector<Quaternion<> > q1Vec( n );
    std::vector<Vector3D<> > pDist( n );
    std::vector<Transform3D<> > bTe( n );

    double maxLength = 0;
    for( size_t j=0; j<n; j++){
        bTe[j] = _fkranges[j]->get(state);
        Quaternion<> q1( bTe[j].R() );
        q1Vec[j] = q1;
        Quaternion<> q2( bTed[j].R() );
        qDist[j] = q2-q1;
        pDist[j] = bTed[j].P() - bTe[j].P();
        double len = qDist[j].getLength();
        if( len>maxLength )
            maxLength = len;
    }

    int steps = (int)ceil( maxLength/_maxQuatStep );
    std::cout << "Steps: " << steps << std::endl;
    double stepSize = 1.0 / (double)steps;
    std::vector<Transform3D<> > target( n );
    std::cout << "StepSize: " << stepSize << std::endl;
    std::cout << "MaxLength: " << maxLength << std::endl;

    //for(double nStep=stepSize; nStep < 0.99999; nStep += stepSize){
    for(int step=1; step < steps; step++){
        double nStep = ((double)step) / (double)steps;

        std::cout << "NStep: " << nStep << std::endl;
        // create target based on quaternion interpolation
        for(size_t j=0; j<n; j++){
            //Quaternion<> qNext = q1Vec[j] + (qDist[j]*nStep);
            //std::cout << "init q: " << q1Vec[j] << std::endl;
            Quaternion<> qNext = qDist[j];
            qNext *= nStep;
            qNext = q1Vec[j] + qNext;
            qNext.normalize();
            //std::cout << "qNext: " << qNext << std::endl;
            //std::cout << "qGoal: " << Quaternion<>(bTed[j].R()) << std::endl;
            Vector3D<> pNext = bTe[j].P() + (pDist[j]*nStep);
            //std::cout << "init p: " << bTe[j].P() << std::endl;
            //std::cout << "pNext: " << pNext << std::endl;
            //std::cout << "pGoal: " << bTed[j].P() << std::endl;
            Transform3D<> bTedTmp(pNext,qNext);
            target[j] = bTedTmp;
        }


        // we allow a relative large error since its only via points
        //std::cout << qNext << " " << pNext << std::endl;
        bool found = performLocalSearch(_device, _fkranges, _jacCalc,
                                        target, maxError, state, maxIterations, 1.0 );

        if(!found)
            return std::vector<Q>();

    }
    */
    // now we perform yet another newton search with higher precision to determine
    // the end result
    if( performLocalSearch(_device, _fkranges, *_jacCalc,
                           bTed, maxError, state, maxIterations, 1.0 ) ){
        //std::cout << "result found!" << std::endl;
        std::vector<Q> result;
        result.push_back(_device->getQ(state));
        return result;
    }
    return std::vector<Q>();
}
#endif
