/********************************************************************************
 * Copyright 2014 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWSIMLIBS_TNTPHYSICS_TNTCOLLISIONSOLVERCHAIN_HPP_
#define RWSIMLIBS_TNTPHYSICS_TNTCOLLISIONSOLVERCHAIN_HPP_

/**
 * @file TNTCollisionSolverChain.hpp
 *
 * \copydoc rwsimlibs::tntphysics::TNTCollisionSolverChain
 */

#include "TNTCollisionSolver.hpp"

namespace rwsimlibs {
namespace tntphysics {

// Forward declarations
class TNTCollisionSolverSingle;
class TNTBody;
class TNTFixedBody;

//! @addtogroup rwsimlibs_tntphysics

//! @{
/**
 * @brief A collision solver that can handle impulse chains.
 *
 * The solver uses the TNTCollisionSolverSingle to solve for individual pairs of objects, and
 * solves iteratively for the objects in the chain as the impulses propagate back and forth.
 *
 * An exception will be thrown if the bodies are connected in a circular or tree type structure.
 *
 * The impulse must either origin from only one contact pair in the chain, or the contact pairs must not
 * share objects. Otherwise an exception will be thrown as it is not possible to simultaneously change the
 * velocity of an object if it is involved in collisions with more than one object.
 */
class TNTCollisionSolverChain: public rwsimlibs::tntphysics::TNTCollisionSolver {
public:
    //! @brief Empty constructor.
	TNTCollisionSolverChain();

    //! @brief Destructor.
	virtual ~TNTCollisionSolverChain();

	//! @copydoc TNTCollisionSolver::doCollisions
	virtual void doCollisions(
			const std::vector<const TNTContact*>& contacts,
			const TNTBodyConstraintManager& bc,
			const TNTMaterialMap* map,
			TNTIslandState& tntstate,
			const rw::kinematics::State& rwstate,
			const rw::common::PropertyMap& pmap,
			rw::common::Ptr<rw::common::ThreadTask> task = NULL) const;

	/**
	 * @copybrief TNTCollisionSolver::addDefaultProperties
	 *
	 * This implementation uses TNTCollisionSolverSingle to solve for pairs of bodies.
	 * See documentation for this class for available properties.
	 *
	 *  Property Name                                         | Type   | Default value | Description
	 *  ----------------------------------------------------- | ------ | ------------- | -----------
	 *  TNTCollisionSolverPropagateThresholdContact           | double | \f$10^{-4}\f$ | Continue propagating impulses as long as there are contacts in chain with relative normal velocities greater than this threshold (m/s).
	 *  TNTCollisionSolverPropagateThresholdConstraintLinear  | double | \f$10^{-8}\f$ | Continue propagating impulses as long as there are constraints in chain with relative linear velocities greater than this threshold (m/s).
	 *  TNTCollisionSolverPropagateThresholdConstraintAngular | double | \f$10^{-8}\f$ | Continue propagating impulses as long as there are constraints in chain with relative angular velocities greater than this threshold (rad/s).
	 *  TNTCollisionSolverMaxIterations                       | int    |     1000      | If impulses are still propagating after this number of iterations, an exception is thrown.
	 *  -                                                     | -      | -             | TNTCollisionSolverSingle::addDefaultProperties defines more properties used by this solver.
	 *
	 * @param map [in/out] the map to add properties to.
	 */
	virtual void addDefaultProperties(rw::common::PropertyMap& map) const;

private:
	struct SharedInfo;
	class DecomposeTask;
	class ChainTask;

	struct Chain {
		std::set<const TNTFixedBody*> fixedBodiesBegin;
		std::vector<const TNTBody*> bodies;
		std::set<const TNTFixedBody*> fixedBodiesEnd;
	};

	static Chain constructChain(
			const std::vector<const TNTContact*>& contacts,
			const TNTBodyConstraintManager& component,
			const TNTIslandState& tntstate);

	static std::set<std::size_t> getIndices(
		const std::vector<const TNTContact*>& contacts,
		const Chain& chain);

	TNTCollisionSolverSingle* _solver;
};
//! @}
} /* namespace tntphysics */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_TNTPHYSICS_TNTCOLLISIONSOLVERCHAIN_HPP_ */
