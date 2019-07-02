/********************************************
 * INVKIN
 ********************************************/

/**
 * @brief Interface for inverse kinematics algorithms
 *
 * The InvKinSolver interface provides an interface for calculating
 * the inverse kinematics of a device.
 *
 * By default it solves the problem beginning at the robot base and
 * ending with the frame defined as the end of the devices, and which is
 * accessible through the Device::getEnd() method.
 */
class InvKinSolver
{
public:
    /**
     * @brief Calculates the inverse kinematics
     *
     * Given a desired transform
     * and the current state, the method solves the inverse kinematics
     * problem. 
     *
     * If the algorithm is able to identify multiple solutions (e.g. elbow
     * up and down) it will return all of these. Before returning a solution,
     * they may be checked to be within the bounds of the configuration space. 
     * (See setCheckJointLimits(bool) )
     *
     * @param baseTend [in] Desired base to end transformation.
     *
     * @param state [in] State of the device from which to start the
     * iterations
     *
     * @return List of solutions. Notice that the list may be empty.
     *
     * @note The targets \b baseTend must be defined relative to the base of the
     * robot/device.
     */
    virtual std::vector<rw::math::Q> solve(const rw::math::Transform3D<double> & baseTend, const State& state) const = 0;

    /**
     * @brief Specifies whether to check joint limits before returning a solution.
     *
     * @param check [in] If true the method should perform a check that joints are within bounds.
     */
    virtual void setCheckJointLimits(bool check) = 0;

    /**
     * @brief Returns the Tool Center Point (TCP) used when solving the IK problem.
     *
     * @return The TCP Frame used when solving the IK.
     */
    virtual rw::common::Ptr< const Frame > getTCP() const = 0;
};

%template (InvKinSolverPtr) rw::common::Ptr<InvKinSolver>;

/**
 * @brief Interface for iterative inverse kinematics algorithms
 *
 * The IterativeIK interface provides an interface for calculating
 * the inverse kinematics of a device.
 *
 * By default it solves the problem beginning at the robot base and
 * ending with the frame defined as the end of the devices, and which is
 * accessible through the Device::getEnd() method.
 */
class IterativeIK: public InvKinSolver
{
public:
    /**
     * @brief Sets the maximal error for a solution
     *
     * The error between two transformations is defined as the maximum of infinite-norm
     * of the positional error and the angular error encoded as EAA.
     *
     * @param maxError [in] the maxError. It will be assumed that maxError > 0
     */
    virtual void setMaxError(double maxError);

    /**
     * @brief Returns the maximal error for a solution
     *
     * @return Maximal error
     */
    virtual double getMaxError() const;

    /**
     * @brief Sets the maximal number of iterations allowed
     *
     * @param maxIterations [in] maximal number of iterations
     */
    virtual void setMaxIterations(int maxIterations);

	/**
     * @brief Returns the maximal number of iterations
     */
    virtual int getMaxIterations() const;

    /**
     * @brief Returns the PropertyMap
     *
     * @return Reference to the PropertyMap
     */
    virtual PropertyMap& getProperties();
    
#if !defined(SWIGJAVA)
    /**
     * @brief Returns the PropertyMap
     *
     * return Reference to the PropertyMap
     */
    virtual const PropertyMap& getProperties() const;
#endif

    /**
       @brief Default iterative inverse kinematics solver for a device and
       state.

       @param device [in] Device for which to solve IK.
       @param state [in] Fixed state for which IK is solved.
    */
    static rw::common::Ptr<IterativeIK> makeDefault(rw::common::Ptr<Device> device, const State& state);
};

%template (IterativeIKPtr) rw::common::Ptr<IterativeIK>;
OWNEDPTR(IterativeIK);

/**
 * \brief A Jacobian based iterative inverse kinematics algorithm for devices with a single end effector.
 *
 * This algorithm does implicitly handle joint limits, however it is possible to force the solution within joint
 * limits using clamping in each iterative step. If joint clamping is not enabled then this
 * algorithm might contain joint values that are out of bounds.
 *
 * The method uses an Newton-Raphson iterative approach and is based on using the inverse of
 * the device Jacobian to compute each local solution in each iteration. Several methods for
 * calculating/approximating the inverse Jacobian are available, where the SVD method currently is
 * the most stable, see the JacobianSolverType option for additional options.
 */
class JacobianIKSolver : public IterativeIK
{
public:
    //! @brief the type of jacobian solver
    typedef enum{Transpose, SVD, DLS, SDLS} JacobianSolverType;

    /**
     * @brief Constructs JacobianIKSolver for device \b device.
     *
     * @param device [in] the device to do inverse kinematics for.
     * @param state [in] the initial state.
     */
    JacobianIKSolver(rw::common::Ptr<const Device> device, const State& state);

    /**
     * @brief Constructs JacobianIKSolver for device, where the frame \b foi will
     * be used as end effector.
     *
     * @param device [in] the device to do inverse kinematics for.
     * @param foi [in] end effector frame.
     * @param state [in] the initial state.
     */
    JacobianIKSolver(rw::common::Ptr<const Device> device, const Frame *foi, const State& state);

    /**
     * @brief Calculates the inverse kinematics
     *
     * Given a desired transform
     * and the current state, the method solves the inverse kinematics
     * problem. 
     *
     * If the algorithm is able to identify multiple solutions (e.g. elbow
     * up and down) it will return all of these. Before returning a solution,
     * they may be checked to be within the bounds of the configuration space. 
     * (See setCheckJointLimits(bool) )
     *
     * @param baseTend [in] Desired base to end transformation.
     *
     * @param state [in] State of the device from which to start the
     * iterations
     *
     * @return List of solutions. Notice that the list may be empty.
     *
     * @note The targets \b baseTend must be defined relative to the base of the
     * robot/device.
     */
    std::vector<rw::math::Q> solve(const rw::math::Transform3D<double> & baseTend, const State& state) const;

    /**
     * @brief sets the maximal step length that is allowed on the
     * local search towards the solution.
     *
     * @param interpolatorStep [in] the interpolation step.
     */
    void setInterpolatorStep(double interpolatorStep);

    /**
     * @brief the solver may fail or be very slow if the the solution is too far from the
     * initial configuration. This function enables the use of via points generated using
     * an interpolation from initial end effector configuration to the goal target.
     *
     * @param enableInterpolation [in] set true to enable interpolation, false otherwise
     */
    void setEnableInterpolation(bool enableInterpolation);

    /**
      * @brief performs a local search toward the target bTed. No via points
      * are generated to support the convergence and robustness.
      *
      * @param bTed [in] the target end pose
      * @param maxError [in] the maximal allowed error
      * @param state [in/out] the starting position for the search. The end position will
      * also be saved in this state.
      * @param maxIter [in] max number of iterations
      * @return true if error is below max error
      * @note the result will be saved in state
      */
    bool solveLocal(const rw::math::Transform3D<double>  &bTed,
                     double maxError,
                     State &state,
                     int maxIter) const;

     /**
      * @brief enables clamping of the solution such that solution always is within joint limits
      *
      * @param enableClamping [in] true to enable clamping, false otherwise
      */
    void setClampToBounds(bool enableClamping);

    /**
     * @brief set the type of solver to use for stepping toward a solution
     *
     * @param type [in] the type of jacobian solver
     */
    void setSolverType(JacobianSolverType type);

    /**
     * @brief Specifies whether to check joint limits before returning a solution.
     *
     * @param check [in] If true the method should perform a check that joints are within bounds.
     */
    void setCheckJointLimits(bool check);

};

%template (JacobianIKSolverPtr) rw::common::Ptr<JacobianIKSolver>;
OWNEDPTR(JacobianIKSolver);

/**
 * @brief Solve the inverse kinematics problem with respect to joint limits and
 * collisions.
 *
 * Given an arbitrary iterative inverse kinematics solver, the IKMetaSolver
 * attempts to find a collision free solution satisfying joint limits. It
 * repeatingly calls the iterative solver with new random start configurations
 * until either a solution is found or a specified max attempts has been
 * reached.
 *
 * Usage example:
 * \code
 * // create a inverse kinematics solver for your dvs. here we use ResolvedRateSolver
 * ResolvedRateSolver iksolver(&myDevice); // takes a pointer to your device
 * // if we want colision free ik results then create or get the collisiondetector
 * CollisionDetector *detector = NULL; // here we don't care about collisions
 * // now create the meta solver
 * IKMetaSolver mSolver(&iksolver, &myDevice, detector);
 * // the pose that you want the endeffector to be in
 * Transform3D<> pose(Vector3D<>(0,0,1),RPY<>(1,0,0));
 * // and use it to generate joint configurations
 * std::vector<Q> result;
 * result = mSolver.solve( pose , state, 200, true );
 * \endcode
 *
 */
class IKMetaSolver: public IterativeIK
{
public:
    /**
     * @brief Constructs IKMetaSolver
     *
     * The IKMetaSolver takes ownership of the \b iksolver. The IKMetaSolver
     * does NOT take ownership of the \b collisionDetector. To skip testing for
     * collision use null as collision detector
     *
     * @param iksolver [in] The basic iksolver to use. Ownership is taken
     * @param device [in] Device to solve for
     *
     * @param collisionDetector [in] CollisionDetector to use. If null no
     * collision detection used.
     */
    IKMetaSolver(rw::common::Ptr<IterativeIK> iksolver,
        const rw::common::Ptr<Device> device,
        rw::common::Ptr<CollisionDetector> collisionDetector = NULL);

    /**
     * @brief Constructs IKMetaSolver
     *
     * The IKMetaSolver takes ownership of the \b iksolver. To skip testing for
     * feasibility set constraint to NULL.
     *
     * @param iksolver [in] The basic iksolver to use. Ownership is taken
     * @param device [in] Device to solve for
     *
     * @param constraint [in] QConstraint pointer to use. If null no
     * constraints is applied
     */
    IKMetaSolver(rw::common::Ptr<IterativeIK> iksolver,
        const rw::common::Ptr<Device> device,
        rw::common::Ptr<QConstraint> constraint);

    /**
     * @brief Calculates the inverse kinematics
     *
     * Given a desired transformation
     * and the current state, the method solves the inverse kinematics
     * problem. 
     *
     * If the algorithm is able to identify multiple solutions (e.g. elbow
     * up and down) it will return all of these. Before returning a solution,
     * they may be checked to be within the bounds of the configuration space. 
     * (See setCheckJointLimits(bool) )
     *
     * @param baseTend [in] Desired base to end transformation.
     *
     * @param state [in] State of the device from which to start the
     * iterations
     *
     * @return List of solutions. Notice that the list may be empty.
     *
     * @note The targets \b baseTend must be defined relative to the base of the
     * robot/device.
     *
     * Searches for a valid solution using the parameters set in the IKMetaSolver
     */
    std::vector<rw::math::Q> solve(const rw::math::Transform3D<double> & baseTend, const State& state) const;

    /**
     * @brief Sets up the maximal number of attempts
     *
     * @param maxAttempts [in] Maximal number of attempts
     */
    void setMaxAttempts(size_t maxAttempts);

    /**
     * @brief Sets whether to stop searching after the first solution
     *
     * @param stopAtFirst [in] True to stop after first solution has been found
     */
    void setStopAtFirst(bool stopAtFirst);

    /**
     * @brief Sets the distance for which two solutions are considered the same.
     *
     * For distance measure an infinite norm is used. Default value is set to 1e-5.
     *
     * Set \b limit < 0 to allow doublets in the solution.
     *
     * @param limit [in] The proximity limit.
     */
    void setProximityLimit(double limit);

    /**
     * @brief Specifies whether to check joint limits before returning a solution.
     *
     * @param check [in] If true the method should perform a check that joints are within bounds.
     */
    void setCheckJointLimits(bool check);

    /**
     * @brief Solves the inverse kinematics problem
     *
     * Tries to solve the inverse kinematics problem using the iterative
     * inverse kinematics solver provided in the constructor. It tries at
     * most \b cnt times and can either be stopped after the first solution
     * is found or continue to search for solutions. If multiple solutions
     * are returned there might be duplicates in the list.
     *
     * @param baseTend [in] Desired base to end transform
     * @param state [in] State of the workcell
     * @param cnt [in] Maximal number of attempts
     *
     * @param stopatfirst [in] If true the method will return after the first
     * solution is found. If false it will continue searching for more solution
     * until the maximal number of attemps is met.
     */
    std::vector<rw::math::Q> solve(const rw::math::Transform3D<double> & baseTend, const State& state, size_t cnt, bool stopatfirst) const;

};

%template (IKMetaSolverPtr) rw::common::Ptr<IKMetaSolver>;
OWNEDPTR(IKMetaSolver);

/**
 * @brief Interface for closed form inverse kinematics algorithms.
 *
 * The ClosedFormIK interface provides an interface for calculating the
 * inverse kinematics of a device.
 *
 * By default it solves the problem beginning at the robot base and
 * ending with the frame defined as the end of the devices, and which is
 * accessible through the Device::getEnd() method.
 */
class ClosedFormIK: public InvKinSolver
{
public:
    /**
       @brief Closed-form IK solver for a device.

       The device must be a serial device with 6 revolute joints described
       by DH parameters.

       The IK solver is currently implemented in terms of PieperSolver. See
       the documentation of PieperSolver for the specific requirements for
       the DH parameters.

       An exception is thrown if closed-form IK for the device is not
       supported, except that all such cases are currently not discovered.
       You should check for yourself that the closed-form IK for the device
       is correct.
    */
    static rw::common::Ptr<ClosedFormIK> make(const Device& device, const State& state);
};

/**
 * @brief Calculates the closed form inverse kinematics of
 * a device using Piepers method
 *
 * To use Piepers method it is required that the device has
 * 6 DOF revolute joints, and that last three axis intersects.
 * In this implementation it will be assumed that the that
 * rotation of these last three axis are equivalent to an
 * Euler ZYZ or Z(-Y)Z rotation.
 *
 * See Introduction to Robotics Mechanics and Control, by
 * John J. Craig for further information about the algorithm.
 */
class PieperSolver: public ClosedFormIK {
public:
    /**
     * @brief Constructor
     *
     * @param dhparams [in] DH-parameters corresponding to the device
     * @param joint6Tend [in] transform from the 6th joint to the end of the device
     * @param baseTdhRef [in] Transformation between the robot base and the reference frame for the DH-parameters.
     */
    PieperSolver(const std::vector<DHParameterSet>& dhparams,
                 const rw::math::Transform3D<double> & joint6Tend,
                 const rw::math::Transform3D<double> & baseTdhRef = rw::math::Transform3D<double> ::identity());

    /**
     * @brief Constructor - the DH parameters is expected to be on each joint
     * in the serial device. When specifying the DH params in the workcell file
     * this constructor can be used.
     *
     * @param dev [in] the device for which to extract the DH parameters.
     * @param joint6Tend [in] transform from the 6th joint to the end of the device
     * @param state [in] State using which the transformation between robot base and the DH-parameters reference frame are calculated.
     * @note throws an exception if the device has no DH params
     */
    PieperSolver(SerialDevice& dev, const rw::math::Transform3D<double> & joint6Tend, const State& state);

    /**
     * @brief Calculates the inverse kinematics
     *
     * Given a desired transformation
     * and the current state, the method solves the inverse kinematics
     * problem. 
     *
     * If the algorithm is able to identify multiple solutions (e.g. elbow
     * up and down) it will return all of these. Before returning a solution,
     * they may be checked to be within the bounds of the configuration space. 
     * (See setCheckJointLimits(bool) )
     *
     * @param baseTend [in] Desired base to end transformation.
     *
     * @param state [in] State of the device from which to start the
     * iterations
     *
     * @return List of solutions. Notice that the list may be empty.
     *
     * @note The targets \b baseTend must be defined relative to the base of the
     * robot/device.
     */
    virtual std::vector<rw::math::Q> solve(const rw::math::Transform3D<double> & baseTend, const State& state) const;

    /**
     * @brief Specifies whether to check joint limits before returning a solution.
     *
     * @param check [in] If true the method should perform a check that joints are within bounds.
     */
    virtual void setCheckJointLimits(bool check);

    /**
     * @brief Returns the Tool Center Point (TCP) used when solving the IK problem.
     *
     * @return The TCP Frame used when solving the IK.
     */
    virtual rw::common::Ptr< const Frame > getTCP() const;

};

%template (ClosedFormIKPtr) rw::common::Ptr<ClosedFormIK>;

/**
 * @brief Analytical inverse solver for the Kuka LBR IIWA 7 R800 robot.
 *
 * Notice that this is a 7 DOF robot and that there is an infinite number of solutions.
 * The extra DOF means that the middle joint of the robot is able to move in a circle.
 * This solver will choose a point on this circle randomly and return up to 8 possible solutions.
 */
class ClosedFormIKSolverKukaIIWA: public ClosedFormIK {
public:
	/**
	 * @brief Construct new closed form solver for a Kuka 7 DOF IIWA robot.
	 *
	 * @param device [in] the device.
	 * @param state [in] the state to get the frame structure and extract the dimensions from.
	 */
	ClosedFormIKSolverKukaIIWA(const rw::common::Ptr<const SerialDevice> device, const State& state);

	//! @brief Destructor.
	virtual ~ClosedFormIKSolverKukaIIWA();

    /**
     * @brief Calculates the inverse kinematics
     *
     * Given a desired transformation
     * and the current state, the method solves the inverse kinematics
     * problem. 
     *
     * If the algorithm is able to identify multiple solutions (e.g. elbow
     * up and down) it will return all of these. Before returning a solution,
     * they may be checked to be within the bounds of the configuration space. 
     * (See setCheckJointLimits(bool) )
     *
     * @param baseTend [in] Desired base to end transformation.
     *
     * @param state [in] State of the device from which to start the
     * iterations
     *
     * @return List of solutions. Notice that the list may be empty.
     *
     * @note The targets \b baseTend must be defined relative to the base of the
     * robot/device.
     */
    std::vector<rw::math::Q> solve(const rw::math::Transform3D<double>& baseTend, const State& state) const;

    /**
     * @brief Find inverse kinematic solutions deterministically by pulling joint 4 as much in the given direction as possible.
     *
     * @param baseTend [in] Desired base to end transformation.
     * @param state [in] State of the device from which to start the iterations.
     * @param dir4 [in] unit vector giving the direction to pull joint 4 in (given in base coordinate system).
     * @return List of up to 8 solutions. Notice that the list may be empty.
     */
    std::vector<rw::math::Q> solve(const rw::math::Transform3D<double>& baseTend, const State& state, const rw::math::Vector3D<double>& dir4) const;

    /**
     * @brief Specifies whether to check joint limits before returning a solution.
     *
     * @param check [in] If true the method should perform a check that joints are within bounds.
     */
    void setCheckJointLimits(bool check);

    /**
     * @brief Returns the Tool Center Point (TCP) used when solving the IK problem.
     *
     * @return The TCP Frame used when solving the IK.
     */
    virtual rw::common::Ptr< const Frame > getTCP() const;
};

%template (ClosedFormIKSolverKukaIIWAPtr) rw::common::Ptr<ClosedFormIKSolverKukaIIWA>;

/**
 * @brief Analytical inverse kinematics solver to the kinematics of a Universal Robots.
 */
class ClosedFormIKSolverUR: public ClosedFormIK {
public:
	/**
	 * @brief Construct new closed form solver for a Universal Robot.
	 *
	 * @note The dimensions will be automatically extracted from the device, using an arbitrary state.
	 *
	 * @param device [in] the device.
	 * @param state [in] the state to use to extract dimensions.
	 */
	ClosedFormIKSolverUR(const rw::common::Ptr<const SerialDevice> device, const State& state);

	//! @brief Destructor.
	virtual ~ClosedFormIKSolverUR();

    /**
     * @brief Calculates the inverse kinematics
     *
     * Given a desired transformation
     * and the current state, the method solves the inverse kinematics
     * problem. 
     *
     * If the algorithm is able to identify multiple solutions (e.g. elbow
     * up and down) it will return all of these. Before returning a solution,
     * they may be checked to be within the bounds of the configuration space. 
     * (See setCheckJointLimits(bool) )
     *
     * @param baseTend [in] Desired base to end transformation.
     *
     * @param state [in] State of the device from which to start the
     * iterations
     *
     * @return List of solutions. Notice that the list may be empty.
     *
     * @note The targets \b baseTend must be defined relative to the base of the
     * robot/device.
     */
    std::vector<rw::math::Q> solve(const rw::math::Transform3D<double>& baseTend, const State& state) const;

    /**
     * @brief Specifies whether to check joint limits before returning a solution.
     *
     * @param check [in] If true the method should perform a check that joints are within bounds.
     */
    void setCheckJointLimits(bool check);

    /**
     * @brief Returns the Tool Center Point (TCP) used when solving the IK problem.
     *
     * @return The TCP Frame used when solving the IK.
     */
    virtual rw::common::Ptr< const Frame > getTCP() const;
};

%template (ClosedFormIKSolverURPtr) rw::common::Ptr<ClosedFormIKSolverUR>;
%template (ClosedFormIKSolverURCPtr) rw::common::Ptr<const ClosedFormIKSolverUR>;