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

#ifndef RWSIMLIBS_RWPE_RWPEISLAND_HPP_
#define RWSIMLIBS_RWPE_RWPEISLAND_HPP_

/**
 * @file RWPEIsland.hpp
 *
 * \copydoc rwsimlibs::rwpe::RWPEIsland
 */

#include <rw/common/Ptr.hpp>
#include <rwsim/simulator/PhysicsEngine.hpp>

// Forward declarations
namespace rw { namespace common { class ThreadTask; } }
namespace rwsim { namespace contacts { class ContactDetector; } }
namespace rwsim { namespace contacts { class ContactDetectorData; } }
namespace rwsim { namespace dynamics { class DynamicWorkCell; } }

namespace rwsimlibs {
namespace rwpe {

// Forward declarations
class RWPEBroadPhase;
class RWPEBodyConstraintGraph;
class RWPEConstraintCorrection;
class RWPEIslandState;
class RWPEMaterialMap;

//! @addtogroup rwsimlibs_rwpe

//! @{
/**
 * @brief The RWPEIsland engine.
 *
 * A RWPEIsland is a basic engine that will solve for all bodies, constraints and contacts in one
 * large equation system. If the system can be decomposed into smaller independent RWPEIslands,
 * it is the responsibility of the user to do this.
 *
 * Under normal circumstances this engine should not be used by the user directly, as it will be
 * too computationally expensive to solve dynamics for large dynamic workcells as one unit.
 * Instead the RWPEWorld engine should be used, which will decompose and manage independent
 * RWPEIslands dynamically to achieve best possible performance. It will also manage time
 * synchronization between islands.
 *
 * Consider using the RWPEIsland directly in rare circumstances if the dynamic system is extremely
 * simple, and the overhead in managing independent islands would be too big.
 *
 * Please note that the simulator uses many thresholds, tolerances and other properties. These
 * are documented in the documentation for the #getDefaultPropertyMap function.
 */
class RWPEIsland: public rwsim::simulator::PhysicsEngine {
public:
    //! Smart pointer type of RWPEIsland
    typedef rw::common::Ptr<RWPEIsland> Ptr;

	//! @brief Create a new empty RWPEIsland.
	RWPEIsland();

	/**
	 * @brief Create a new empty RWPEIsland.
	 * @param detector [in] the contact detector to use.
	 */
	RWPEIsland(rw::common::Ptr<rwsim::contacts::ContactDetector> detector);

	/**
	 * @brief Create a new RWPEIsland physics engine based on a complete dynamic workcell.
	 * @param dwc [in] the dynamic workcell to create island from.
	 * @param detector [in] (optional) the contact detector to use. If none given a default detector is created.
	 */
	RWPEIsland(rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> dwc, rw::common::Ptr<rwsim::contacts::ContactDetector> detector = NULL);

	//! @brief Destructor
	~RWPEIsland();

	/**
	 * @brief Get the currently used gravity in world frame.
	 * @return the gravity.
	 */
	const rw::math::Vector3D<>& getGravity() const;

	/**
	 * @brief Set the gravity.
	 * @param gravity [in] the gravity given in world frame.
	 */
	void setGravity(const rw::math::Vector3D<> &gravity);

	/**
	 * @brief Step the simulator by adding work to ThreadTask.
	 *
	 * The work is added as a single subtask to the given ThreadTask.
	 * @note This functions returns immediately and the state should not be accessed
	 * before work added to ThreadTask has finished.
	 *
	 * @param dt [in] the timestep.
	 * @param task [in] the task to add work to.
	 */
	void step(double dt, rw::common::Ptr<rw::common::ThreadTask> task);

	/**
	 * @brief Get a copy of the internal state of the engine.
	 * @return a copy of the state.
	 */
	RWPEIslandState getRWPEState() const;

	/**
	 * @brief Get the current internal state of the simulator.
	 * @return a copy of the internal state.
	 */
	rw::kinematics::State getState() const;

	/**
	 * @brief Reset the internal engine state.
	 * @param state [in] the state to reset to.
	 */
	void resetScene(const RWPEIslandState &state);

	//! @copydoc PhysicsEngine::load
	virtual void load(rwsim::dynamics::DynamicWorkCell::Ptr dwc);

	//! @copydoc PhysicsEngine::setContactDetector
	bool setContactDetector(rw::common::Ptr<rwsim::contacts::ContactDetector> detector);

	//! @copydoc PhysicsEngine::step
	void step(double dt);

	//! @copydoc PhysicsEngine::step
	void step(double dt, rw::kinematics::State& state);

	//! @copydoc PhysicsEngine::resetScene
	void resetScene(rw::kinematics::State& state);

	//! @copydoc PhysicsEngine::initPhysics
	void initPhysics(rw::kinematics::State& state);

	//! @copydoc PhysicsEngine::exitPhysics
	void exitPhysics();

	//! @copydoc PhysicsEngine::getTime
	double getTime();

	//! @copydoc PhysicsEngine::setEnabled
	void setEnabled(rwsim::dynamics::Body::Ptr body, bool enabled);

	//! @copydoc PhysicsEngine::setDynamicsEnabled
	void setDynamicsEnabled(rwsim::dynamics::Body::Ptr body, bool enabled);

	//! @copydoc PhysicsEngine::createDebugRenderer
	rwsim::drawable::SimulatorDebugRender::Ptr createDebugRender();

	//! @copydoc PhysicsEngine::getPropertyMap
	rw::common::PropertyMap& getPropertyMap();

	//! @copydoc PhysicsEngine::getPropertyMap
	const rw::common::PropertyMap& getPropertyMap() const;

	/**
	 * @brief Get a map with the default properties.
	 * @return default PropertyMap.
	 */
	const rw::common::PropertyMap& getPropertyMapDefault() const;

	//! @copydoc PhysicsEngine::emitPropertyChanged
	void emitPropertyChanged();

	//! @copydoc PhysicsEngine::addController
	void addController(rwlibs::simulation::SimulatedController::Ptr controller);

	//! @copydoc PhysicsEngine::removeController
	void removeController(rwlibs::simulation::SimulatedController::Ptr controller);

	//! @copydoc PhysicsEngine::addBody
	void addBody(rwsim::dynamics::Body::Ptr body, rw::kinematics::State &state);

	//! @copydoc PhysicsEngine::addDevice
	void addDevice(rwsim::dynamics::DynamicDevice::Ptr dev, rw::kinematics::State &state);

	//! @copydoc PhysicsEngine::addSensor
	void addSensor(rwlibs::simulation::SimulatedSensor::Ptr sensor, rw::kinematics::State &state);

	//! @copydoc PhysicsEngine::removeSensor
	void removeSensor(rwlibs::simulation::SimulatedSensor::Ptr sensor);

	//! @copydoc PhysicsEngine::attach
	void attach(rwsim::dynamics::Body::Ptr b1, rwsim::dynamics::Body::Ptr b2);

	//! @copydoc PhysicsEngine::detach
	void detach(rwsim::dynamics::Body::Ptr b1, rwsim::dynamics::Body::Ptr b2);

	//! @copydoc PhysicsEngine::getSensors
	std::vector<rwlibs::simulation::SimulatedSensor::Ptr> getSensors();

	//! @copydoc PhysicsEngine::setSimulatorLog
	void setSimulatorLog(rw::common::Ptr<rwsim::log::SimulatorLogScope> log);

	/**
	 * @brief Get a new PropertyMap with default properties.
	 *
	 * This function will return the properties that are used if they have not been
	 * specified by the user. These options are loaded from the dynamic workcell.
	 * Please see the \ref sec_rwsimxml_physicsengine "Dynamic WorkCell XML File Format" page for
	 * more information on this.
	 *
	 * Below you find a list of properties used by this engine by default. Please be aware that
	 * some properties are used to specify alternative implementations of certain subcomponents of
	 * the simulator. These alternative implementations might use different properties, which will
	 * be documented in the relevant classes. The map returned by this function will include the default
	 * properties for the default choice of subcomponents. These will not be documented in the list below,
	 * but instead references are provided to more information on the specific properties for these components.
	 *
	 *  Property Name           | Type   | Default value | Description
	 *  ----------------------- | ------ | ------------- | -----------
	 *  RWPECollisionSolver     | string | Chain         | Specifies the RWPECollisionSolver used (default is RWPECollisionSolverChain).
	 *  -                       | -      | -             | See RWPECollisionSolverChain::addDefaultProperties for further information on the properties used by collision solvers.
	 *  RWPEContactResolver     | string | Full          | Specifies the RWPEContactResolver used (default is RWPEContactResolverFull).
	 *  -                       | -      | -             | See RWPEContactResolver::addDefaultProperties for further information on the properties used by contact resolvers.
	 *  RWPEConstraintSolver    | string | Iterative     | Specifies the RWPEConstraintSolver used (default is RWPEConstraintSolverIterative).
	 *  -                       | -      | -             | See RWPEConstraintSolver::addDefaultProperties for further information on the properties used by solvers.
	 *  RWPERollbackMethod      | string | Ridder        | Specifies the RWPERollbackMethod used (default is RWPERollbackMethodRidder).
	 *  RWPECorrection          | int    | 1             | Enable or disable correction (default is enabled=1).
	 *  -                       | -      | -             | See RWPEConstraintCorrection::addDefaultProperties for further information on the properties used by correction method.
	 *  RWPERollback            | int    | 1             | Enable or disable rollback (default is enabled=1).
	 *  RWPERollbackThreshold   | float  | \f$10^{-5}\f$ | Precision of rollback.
	 *  RWPERollbackIterations  | int    | 10            | Stop with exception if rollback requires more than this number of iterations.
	 *  RWPEWorkspace           | float  | 10            | The coordinates of all bodies must lie within this workspace, or simulation fails (meters).
	 *  RWPEConstraintMaxForce  | float  | \f$10^{6}\f$  | Stop with exception if theshold exceeds this value (Newtons).
	 */
	static rw::common::PropertyMap getDefaultPropertyMap();

private:
	class StepTask;
	struct IntegrateSample;
	struct IntegrateSampleCompare {
		bool operator()(const IntegrateSample& s1, const IntegrateSample& s2) const;
	};
	typedef std::set<IntegrateSample, IntegrateSampleCompare> IntegrateBuffer;

	void doStep(double dt);
	void velocityAndForce(double dt, bool discontinuity, const IntegrateSample& sample0, IntegrateSample& sampleH) const;
	void position(IntegrateSample& sample, rwsim::contacts::ContactDetectorData& cdData) const;
	bool collisionSolver(RWPEIslandState& islandState, const rw::kinematics::State& rwstate) const;
	double rollbackBroadPhase(double dt, bool discontinuity, const IntegrateSample& first, IntegrateSample& res) const;
	double rollback(bool discontinuity, IntegrateSample& sample0, IntegrateSample& sampleH, rwsim::contacts::ContactDetectorData& cdData) const;
	IntegrateSample rollbackNarrowPhase(bool discontinuity, IntegrateSample& sample0, IntegrateSample& sampleH, rwsim::contacts::ContactDetectorData& cdData) const;
	void constraintSolver(double dt, bool discontinuity, const RWPEIslandState& islandState0, RWPEIslandState& islandStateH, const rw::kinematics::State& rwstate) const;

	void storeResults(rwsim::contacts::ContactDetectorData& cdData, IntegrateSample& sample, rw::kinematics::State& rwstate) const;

private:
	const RWPEConstraintCorrection* const _correction;
	std::vector<rwlibs::simulation::SimulatedController::Ptr> _controllers;

	rw::common::Ptr<const rwsim::dynamics::DynamicWorkCell> _dwc;
	const RWPEMaterialMap* _materialMap;

	RWPEBroadPhase* _bp;
	rw::common::Ptr<rwsim::contacts::ContactDetector> _detector;
	std::list<rwlibs::simulation::SimulatedSensor::Ptr> _sensors;

	rw::common::Ptr<rw::common::ThreadTask> _task;
	RWPEBodyConstraintGraph* _bc;
	RWPEIslandState* _state;
	rw::kinematics::State _rwstate;
	rw::math::Vector3D<> _gravity;

	rwsim::drawable::SimulatorDebugRender::Ptr _render;
	class RWPELogUtil* _log;

	rw::common::PropertyMap _map;
	const rw::common::PropertyMap _defaultMap;
};
//! @}
} /* namespace rwpe */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_RWPE_RWPEISLAND_HPP_ */