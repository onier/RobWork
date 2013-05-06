/*
    Copyright 2013 <copyright holder> <email>

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/


#include "BeamStartGuess.hpp"

#include <rw/math/Rotation2D.hpp>
#include <rw/math/Vector2D.hpp>
#include <rwlibs/softbody/beam/EBBeam.hpp>

#include <rw/common/macros.hpp>

#include <math.h>

using namespace rwlibs::softbody;



 void BeamStartGuess::setZeroStartingGuess ( boost::numeric::ublas::vector< double >& avec, boost::shared_ptr< ModRusselBeamBase > beamPtr )  {
    RW_ASSERT( (int) avec.size() == beamPtr->getM() );
    const int M = beamPtr->getM();
    
    for ( int i = 0; i < M; i++ ) {
        avec[i] = 0.0;
    }
}


 void BeamStartGuess::setEulerStartingGuess ( boost::numeric::ublas::vector< double >& avec, boost::shared_ptr< rwlibs::softbody::BeamGeometryCuboid > beamGeomPtr )  {
    
    // Problem: method on beamPtr returns base class, and we need the cuboid specialization for out parameters to the EB beam
    // could pass reference here in order to access it....this methdo ONLY works for EB
    //const BeamGeometry &geom = _beamPtr->getGeometry();
    const double g2 = -beamGeomPtr->g2();
    const int M = beamGeomPtr->getM();

    EBBeam beam ( beamGeomPtr->getH(), beamGeomPtr->getK(), beamGeomPtr->getL(), 0.5, 1.155e-6, beamGeomPtr->get_h(), g2 );
    for ( int i = 0; i < M; i++ ) {
        avec[i] = atan ( beam.d ( i ) );
    }
}
