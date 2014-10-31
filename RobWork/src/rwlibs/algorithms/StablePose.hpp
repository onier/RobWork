/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#ifndef RWSIM_UTIL_STABLEPOSE_HPP_
#define RWSIM_UTIL_STABLEPOSE_HPP_

#include <rw/math/Vector3D.hpp>
#include <vector>

namespace rwlibs {
namespace algorithms {

    /**
     * @brief A stable pose is a pose in which an object is in rest on top of a supporting surface.
     * In other words an object is in a stable pose if its potential is in a local minimum.
     * Two stable poses might be equal if one can define a continues trajectory from one pose to another
     * such that any pose on that trajectory is a stable pose. By definition all poses on that trajectory will
     * belong to one single stable pose.
     *
     * Often stable poses can be freely rotated around a single axis. Consider an object on a table, then the
     * rotation axis will be the normal to the table. We say that the stable pose is invariant to rotations in this
     * axis.
     *
     */
	class StablePose {
	public:

	    /**
	     * @brief define a stable pose that is invariant in rotAxis. rotAxis is normally defined
	     * in object coordinates, however this might depend on the application.
	     * @param rotAxis [in] invariant rotation axis
	     * @param h [in] height of center of mass of object above support plane
	     */
		StablePose(rw::math::Vector3D<> rotAxis, double h=0):
			_degree(1),
			_rotAxes(1,rotAxis),
			_posAxes(1),
			_rotAxesTable(1),
			_probability(-1),
			_height(h)
		{};

		StablePose(int degree, double prob):
			_degree(degree),
			_rotAxes(1),
			_posAxes(1),
			_rotAxesTable(1),
			_probability(prob)
		{};


		//! destructor
		virtual ~StablePose(){};

		/**
		 * @brief set the probability of object landing in this pose. The distribution is application
		 * dependent so user cannot assume that probability comes from a uniform pose distribution.
		 * @param prop [in] pose distribution relative to application
		 */
		void setProbability(double prop){ _probability = prop; }

        /**
         * @brief get the probability of object landing in this pose. The distribution is application
         * dependent so user cannot assume that probability comes from a uniform pose distribution.
         * @return probability of pose landing in this stable pose [0;1]
         * @note if the probability is less than 0 then it is undefined
         */
		double getProbability(){ return _probability; }

		//! test if this stablepose has a computed probability
		bool hasProbability(){ return _probability>=0; }

		//! get invariant rotation vectors
		std::vector< rw::math::Vector3D<> > getInvariantRotationVectors(){ return _rotAxes; };

		//! get invariant position vectors
		std::vector< rw::math::Vector3D<> > getInvariantPositionVectors(){ return _posAxes; };

		//! get invariant rotation vectors relative to supporting frame
		std::vector< rw::math::Vector3D<> > getInvariantRotationVectorsInSupport();

	private:
		// redundant, since length of _rotAxes is also the degree.
		// though its nice to have
		int _degree;
		// invariant rotation axes
		std::vector< rw::math::Vector3D<> > _rotAxes; // relative to own coordinate frame
        // position
		std::vector< rw::math::Vector3D<> > _posAxes; // position of contact relative to own coordinate frame

		std::vector< rw::math::Vector3D<> > _rotAxesTable; // relative to supporting structures frame

		// each rotation axis can be valid in a number of angle intervals
		std::vector< std::vector<std::pair<double,double> > > _segments;

		// the height from

		//rw::math::Transform3D<> _trans;

		// the statistics
		double _probability;
		double _quality;
		double _height;
	};
}
}
#endif
