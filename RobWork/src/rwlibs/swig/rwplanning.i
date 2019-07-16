/********************************************
 * PATHPLANNING
 ********************************************/

//! @brief Interface for the sampling a configuration.
class QSampler
{
public:
    /**
     * @brief Sample a configuration.
     *
     * If sampling fails, the sampler may return the empty configuration. If
     * empty() is true then the sampler has no more configurations.
     * Otherwise sample() may (or may not) succeed if called a second time.
     */
    rw::math::Q sample();

    /**
     * @brief True if the sampler is known to contain no more
     * configurations.
     */
    bool empty() const;

    /**
     * @brief Destructor
     */
    virtual ~QSampler();

    /**
     * @brief Empty sampler.
     */
	static rw::common::Ptr<QSampler> makeEmpty();

    /**
     * @brief Sampler that always returns the same configuration.
     *
     * The sampler is considered never empty (empty() always returns false).
     */
	static rw::common::Ptr<QSampler> makeFixed(const rw::math::Q& q);

    /**
     * @brief Sampler that always returns a single configuration.
     *
     * The sample() returns \b q the first time the method is called and the
     * empty configuration otherwise. empty() returns true after the first
     * call of sample().
     */
	static rw::common::Ptr<QSampler> makeSingle(const rw::math::Q& q);

    /**
     * @brief Sampler for the values of a finite sequence.
     *
     * sample() returns each of the values of \b qs in order. When all of
     * these samples have been returned, empty() returns true and sample()
     * returns the empty configuration.
     */
	static rw::common::Ptr<QSampler> makeFinite(const std::vector<rw::math::Q>& qs);

    /**
     * @brief A sampler to that returns only the first \b cnt samples from
     * another sampler.
     *
     * The sampler is considered empty as soon as \b sampler is empty or the
     * sampler has been called \b cnt times or more.
     */
	static rw::common::Ptr<QSampler> makeFinite(rw::common::Ptr<QSampler> sampler, int cnt);

    /**
     * @brief Uniform random sampling for a box of the configuration space.
     */
	//static rw::common::Ptr<QSampler> makeUniform(const Device::QBox& bounds);

    /**
     * @brief Uniform random sampling for a device.
     */
	static rw::common::Ptr<QSampler> makeUniform(
        const Device& device);

    /**
     * @brief Uniform random sampling for a device.
     */
	static rw::common::Ptr<QSampler> makeUniform(rw::common::Ptr<const Device> device);

    /**
     * @brief Map a sampler of standard configurations into a sampler of
     * normalized configurations.
     */
	//static rw::common::Ptr<QSampler> makeNormalized(rw::common::Ptr<QSampler> sampler, const QNormalizer& normalizer);

    /**
     * @brief A sampler of IK solutions for a specific target.
     *
     * @param sampler [in] Sampler of IK solutions for \b target.
     * @param target [in] Target for IK solver.
     */
	//static rw::common::Ptr<QSampler> make(rw::common::Ptr<QIKSampler> sampler, const rw::math::Transform3D<>& target);

    /**
     * @brief A sampler filtered by a constraint.
     *
     * For each call of sample() up to \b maxAttempts configurations are
     * extracted from \b sampler and checked by \b constraint. The first
     * sample that satisfies the constraint is returned; if no such were
     * found the empty configuration is returned.
     *
     * If \b maxAttempts is negative (this is the default), then \b sampler
     * is sampled forever until either the \b sampler is empty or a
     * configuration satisfying \b constraint is found.
     */
	//static rw::common::Ptr<QSampler> makeConstrained(
	//	rw::common::Ptr<QSampler> sampler,
	//	rw::common::Ptr<const QConstraint> constraint,
    //    int maxAttempts = -1);

    /**
     * @brief Sampler of direction vectors for a box shaped configuration
     * space.
     *
     * Each random direction vector is found by sampling a configuration
     * uniformly at random from \b bounds, and returning the vector from the
     * center of the box to the configuration. The returned direction vector
     * can therefore be of length zero.
     */
    //static
	//	rw::common::Ptr<QSampler> makeBoxDirectionSampler(
    //    const Device::QBox& bounds);

protected:
    /**
     * @brief Constructor
     */
    QSampler();

    /**
     * @brief Subclass implementation of the sample() method.
     */
    virtual rw::math::Q doSample() = 0;

    /**
     * @brief Subclass implementation of the empty() method.
     *
     * By default the sampler is assumed to be sampling an infinite set of
     * configurations. IOW. the function returns false by default.
     */
    virtual bool doEmpty() const;

private:
    QSampler(const QSampler&);
    QSampler& operator=(const QSampler&);
};
//! @brief smart pointer type to this class
%template (QSamplerPtr) rw::common::Ptr<QSampler>;
//! @brief smart pointer type to this const class
%template (QSamplerCPtr) rw::common::Ptr<const QSampler>;

OWNEDPTR(QSampler)

class PlannerConstraint
{
public:
    PlannerConstraint();

    //PlannerConstraint(QConstraint::Ptr constraint, QEdgeConstraint::Ptr edge);

    bool inCollision(const rw::math::Q& q);

    bool inCollision(const rw::math::Q& q1, const rw::math::Q& q2);

    //QConstraint& getQConstraint() const { return *_constraint; }

    //QEdgeConstraint& getQEdgeConstraint() const { return *_edge; }

    //const QConstraint::Ptr& getQConstraintPtr() const { return _constraint; }

    //const QEdgeConstraint::Ptr& getQEdgeConstraintPtr() const { return _edge; }

    //static PlannerConstraint make(QConstraint::Ptr constraint, QEdgeConstraint::Ptr edge);

    static PlannerConstraint make(rw::common::Ptr<CollisionDetector> detector,
                                  rw::common::Ptr<const Device> device,
                                  const State& state);

    static PlannerConstraint make(rw::common::Ptr<CollisionStrategy> strategy,
                                  rw::common::Ptr<WorkCell> workcell,
                                  rw::common::Ptr<const Device> device,
                                  const State& state);

    /*
    static PlannerConstraint make(rw::proximity::CollisionStrategy::Ptr strategy,
        const rw::proximity::CollisionSetup& setup,
        rw::common::Ptr<WorkCell> workcell,
        rw::common::Ptr<Device> device,
        const State& state);
     */
};

%template (PlannerConstraintPtr) rw::common::Ptr<PlannerConstraint>;
OWNEDPTR(PlannerConstraint);

%nodefaultctor StopCriteria;
class StopCriteria
 {
#if defined(SWIGJAVA)
%apply bool[] {bool *};
#endif
 
 public:
     bool stop() const;
     rw::common::Ptr<StopCriteria> instance() const;
     virtual ~StopCriteria();
     static rw::common::Ptr<StopCriteria> stopAfter(double time);
     static rw::common::Ptr<StopCriteria> stopNever();
     static rw::common::Ptr<StopCriteria> stopNow();
     static rw::common::Ptr<StopCriteria> stopByFlag(bool* stop);
     //static rw::common::Ptr<StopCriteria> stopByFun(boost::function<bool ()> fun);
     static rw::common::Ptr<StopCriteria> stopCnt(int cnt);
     static rw::common::Ptr<StopCriteria> stopEither(
         const std::vector<rw::common::Ptr<StopCriteria> >& criteria);

     static rw::common::Ptr<StopCriteria> stopEither(
         const rw::common::Ptr<StopCriteria>& a,
         const rw::common::Ptr<StopCriteria>& b);
};

%template (StopCriteriaPtr) rw::common::Ptr<StopCriteria>;
%template (StopCriteriaPtrVector) std::vector<rw::common::Ptr<StopCriteria> >;

%nodefaultctor PathPlanner;
template <class From, class To, class PATH = Path<From> >
class PathPlanner
{
public:

    bool query(const From& from, To& to, PATH& path, const StopCriteria& stop);

    bool query(const From& from, To& to, PATH& path, double time);

    bool query(const From& from, To& to, PATH& path);

    PropertyMap& getProperties();

};
%nodefaultctor PathPlannerQQ;
%template (PathPlannerQQ) PathPlanner<rw::math::Q,const rw::math::Q>;
%nodefaultctor PathPlannerQQSampler;
%template (PathPlannerQQSampler) PathPlanner<rw::math::Q, QSampler>;

/**
 * @brief Normalization of configurations.
 *
 * QNormalizer linearly maps configurations of a rectangular configuration
 * space into a square configuration space with lower corner (0, 0, ..., 0)
 * and upper corner (1, 1, ..., 1).
 */
class QNormalizer
{
public:
    /**
     * @brief Convert from a normalized configuration to a real
     * configuration.
     */
    rw::math::Q fromNormalized(const rw::math::Q& q) const;

    /**
     * @brief Convert a real configuration to a normalized configuration.
     */
    rw::math::Q toNormalized(const rw::math::Q& q) const;

    /**
     * @brief Convert from a normalized configuration to a real
     * configuration and assign the real configuration to \b q.
     */
    void setFromNormalized(rw::math::Q& q) const;

    /**
     * @brief Convert a real configuration to a normalized configuration and
     * write the normalized configuration to \b q.
     */
    void setToNormalized(rw::math::Q& q) const;

    /**
     * @brief The box of the configuration space with respect to which
     * normalization is done.
     */
    const std::pair<rw::math::Q, rw::math::Q>& getBounds() const;

    /**
     * @brief Normalizer for the configuration space box given by \b bounds.
     */
    explicit QNormalizer(const std::pair<rw::math::Q, rw::math::Q>& bounds);

    /**
     * @brief Normalizer for the already normalized configuration space box.
     */
    static QNormalizer identity();

private:
    QNormalizer();
};
%template (QNormalizerPtr) rw::common::Ptr<QNormalizer>;
OWNEDPTR(QNormalizer)

/**
   @brief Interface for the checking for collisions for work cell states.
*/
class QConstraint
{
public:
	/**
     * @brief Destructor
     */
    virtual ~QConstraint();

	/**
	 * @brief Set the log to be used for writing debug info
	 * @param log [in] Log to which debug information is to be written
	 */
	virtual void setLog(rw::common::Ptr<Log> log);

	/**
	 * @brief Updates the constraint with a new state
	 * 
	 * The method might not have an effect on all constrainttypes.
	 */
	void update(const State& state);

    /**
     * @brief True if the work cell is considered to be in collision for the
     * device configuration \b q.
     */
    bool inCollision(const rw::math::Q& q) const;

    /**
     * @brief A fixed constraint.
     *
     * The fixed constraint always returns \b value from inCollision().
     */
	static rw::common::Ptr<QConstraint> makeFixed(bool value);

    /**
     * @brief Constraint for the bounds of the configuration space.
     *
     * The configuration is considered to be in collision if it is outside
     * of the bounds given by \b bounds.
     */
	static rw::common::Ptr<QConstraint> makeBounds(const std::pair<rw::math::Q, rw::math::Q>& bounds);

    /**
     * @brief Map a state constraint to a configuration constraint.
     */
	static rw::common::Ptr<QConstraint> make(
		rw::common::Ptr<StateConstraint> detector,
		rw::common::Ptr<const Device> device,
        const State& state);

    /**
     * @brief Map a collision detector to a configuration constraint.
     */
	static rw::common::Ptr<QConstraint> make(
		rw::common::Ptr<CollisionDetector> detector,
		rw::common::Ptr<const Device> device,
        const State& state);

    /**
     * @brief Combine a set of configuration constraints into a single
     * configuration constraint.
     */
	static rw::common::Ptr<QConstraint> makeMerged(
		const std::vector<rw::common::Ptr<QConstraint> >& constraints);

    /**
     * @brief Combine a pair of configuration constraints into a single
     * configuration constraint.
     */
	static rw::common::Ptr<QConstraint> makeMerged(
		const rw::common::Ptr<QConstraint>& ca,
		const rw::common::Ptr<QConstraint>& cb);

    /**
     * @brief Map a configuration constraint for standard configurations
     * into a configuration constraint for normalized configurations.
     *
     * Configuration values are mapped from the range [0, 1] into the
     * corresponding position in the box \b bounds.
     */
	static rw::common::Ptr<QConstraint> makeNormalized(
		const rw::common::Ptr<QConstraint>& constraint,
        const std::pair<rw::math::Q, rw::math::Q>& bounds);

    /**
     * @brief Map a configuration constraint for standard configurations
     * into a configuration constraint for normalized configurations.
     *
     * Configuration values are mapped from the range [0, 1] into the
     * corresponding position in the configuration space of \b device.
     */
	static rw::common::Ptr<QConstraint> makeNormalized(
		const rw::common::Ptr<QConstraint>& constraint,
        const Device& device);

    /**
     * @brief Map a configuration constraint for standard configurations
     * into a configuration constraint for normalized configurations.
     *
     * Configuration values are mapped from normalized configurations into
     * standard configurations using \b normalizer.
     */
	static rw::common::Ptr<QConstraint> makeNormalized(
		const rw::common::Ptr<QConstraint>& constraint,
        const QNormalizer& normalizer);

protected:
    /**
     * @brief Subclass implementation of the inCollision() method.
     */
    virtual bool doInCollision(const Q& q) const = 0;

    /**
     * @brief Set a log.
     *
     * @param log [in] the log.
     */
	virtual void doSetLog(rw::common::Ptr<rw::common::Log> log) = 0;

	/**
	 * @brief Update constraint.
	 *
	 * @param state [in] the state.
	 */
	virtual void doUpdate(const State& state);

    /**
     * Constructor
     */
    QConstraint() {}

private:
    QConstraint(const QConstraint&);
    QConstraint& operator=(const QConstraint&);
};

%template (QConstraintPtr) rw::common::Ptr<QConstraint>;
%template (QConstraintCPtr) rw::common::Ptr<const QConstraint>;
%template (QConstraintPtrVector) std::vector<rw::common::Ptr<QConstraint> >;

/**
 * @brief Edge constraint interface.
 *
 * An edge constraint represents a path that connects a pair of
 * configurations and checks if this path can be traversed.
 *
 * The edge constraint may assume that the start and end configurations are
 * valid (e.g. not colliding).
 *
 * Each edge has a non-negative cost measuring the degree to which the path
 * connecting the configurations has been verified. You can use the cost
 * measure to for example always verify the edge for which the most of the
 * path still remains to be verified. The exact meaning of the cost is
 * defined by the specific subclass.
 *
 * Given an edge constraint you can construct a new edge constraint of the same
 * type, but for a new pair of configurations, with
 * QEdgeConstraint::instance().
 */
class QEdgeConstraint
{
public:
	//! @brief smart pointer type to this class
	typedef rw::common::Ptr<QEdgeConstraint> Ptr;
	//! @brief smart pointer type to this class
	typedef rw::common::Ptr< const QEdgeConstraint > CPtr;

    /**
     * @brief Destructor
     */
    virtual ~QEdgeConstraint();

    /**
     * @brief True if the path from \b start to \b end can't be traversed.
     *
     * @param start [in] Start configuration.
     * @param end [in] End configuration.
     */
    bool inCollision(const rw::math::Q& start, const rw::math::Q& end) const;

    /**
     * @brief Discrete path verification for a linearly interpolated path.
     *
     * Performs a binary style checking of the edge with a resolution of \b resolution.
	 * The length of the edge is virtually extended to exactly match the specified resolution.
	 * However, only configurations within the original length are tested.
	 *
	 * Each configuration tested is checked using \b constraint.
	 *
	 * The metric must be well-behaved, i.e. linear.
	 *
     * Start and end configurations are assumed to be collision free.
     *
	 * @param constraint [in] Constraint to check configurations with
	 * @param metric [in] Metric with which the resolution it to be measured
	 * @param resolution [in] The test resolution
    */
	static rw::common::Ptr<QEdgeConstraint> make(rw::common::Ptr<QConstraint> constraint,
									 rw::common::Ptr<const Metric<rw::math::Q> > metric,
									 double resolution);

    /**
     * @brief Default edge constraint for a configuration constraint and a
     * device.
     *
     * Start and end configurations are connected by a straight line in the
     * configuration space and are checked by a default collision checking
     * resolution.
     */
	static rw::common::Ptr<QEdgeConstraint> makeDefault(rw::common::Ptr<QConstraint> constraint,
		rw::common::Ptr< const Device > device);


	/**
	 * @brief Makes an edge constraint by combining multiple edge constraints
	 *
	 * The constraints provided are called one by one in the order provided.
	 * It is assumed that all constraints matches the same device.
	 *
	 * @param constraints [in] List of constraints to check
	 * @return Pointer to the resulting QEdgeConstraint. Pointer has ownership.
	 **/
	static rw::common::Ptr<QEdgeConstraint> makeMerged(const std::vector<rw::common::Ptr<QEdgeConstraint> >& constraints);

	/**
	 * @brief Makes an edge constraint by combining two edge constraints
	 *
	 * The constraints provided are called one by one in the order provided.
	 * It is assumed that all constraints matches the same device.
	 *
	 * @param constraint1 [in] First constraint to check
	 * @param constraint2 [in] Second constraint to check
	 * @return Pointer to the resulting QEdgeConstraint. Pointer has ownership.
	 **/
	static rw::common::Ptr<QEdgeConstraint> makeMerged(rw::common::Ptr<QEdgeConstraint> constraint1, rw::common::Ptr<QEdgeConstraint> constraint2);


protected:

    /**
     * @brief Subclass implementation of the inCollision() method.
     *
     * By default the method is implemented in terms of instance() and
     * inCollision().
     */
    virtual bool doInCollision(const rw::math::Q& start,
							   const rw::math::Q& end) const = 0;



};

%template (QEdgeConstraintPtr) rw::common::Ptr<QEdgeConstraint>;
%template (QEdgeConstraintPtrVector) std::vector<rw::common::Ptr<QEdgeConstraint> >;

/**
 * @brief Edge constraint interface for incremental testing of an edge
 *
 * An edge constraint represents a path that connects a pair of
 * configurations and checks if this path can be traversed.
 *
 * The edge constraint may assume that the start and end configurations are
 * valid (e.g. not colliding).
 *
 * Each edge has a non-negative cost measuring the degree to which the path
 * connecting the configurations has been verified. You can use the cost
 * measure to for example always verify the edge for which the most of the
 * path still remains to be verified. The exact meaning of the cost is
 * defined by the specific subclass.
 *
 * Given an edge planner you can construct a new edge planner of the same
 * type, but for a new pair of configurations, with
 * QEdgeConstraint::instance().
 */
class QEdgeConstraintIncremental
{
public:
    /**
     * @brief Destructor
     */
    virtual ~QEdgeConstraintIncremental();

    /**
     * @brief True if the path from \b start to \b end can't be traversed.
     *
     * @param start [in] Start configuration.
     * @param end [in] End configuration.
     */
    bool inCollision(
        const rw::math::Q& start,
        const rw::math::Q& end) const;

    /**
     * @brief True if the path connecting the start and end configuration
     * can't be traversed.
     */
    bool inCollision();

    /**
     * @brief Non-negative measure of the amount of the path that still
     * remains to be verified.
     *
     * The exact definition of the cost is decided by the subclass.
     *
     * The cost of an edge should strictly decrease for every call of
     * verifyIncrement().
     *
     * The cost of a fully verified edge can be 0, but does not have to be.
     */
    double inCollisionCost() const;

    /**
     * @brief Perform a partial check of the path and return true if a
     * collision was found.
     *
     * Full check of the path can be implemented in terms of a sequence of
     * partial checks. The isFullyChecked() method returns true when there
     * are no more partial checks to be done.
     */
    bool inCollisionPartialCheck();

    /**
     * @brief True if the path has been fully checked.
     *
     * To check a path, either call inCollision() or repeatedly call
     * inCollisionPartialCheck() until inCollisionPartialCheck() returns
     * false or isFullyChecked() returns true.
     */
    bool isFullyChecked() const;

    /**
     * @brief An edge constraint for a pair of configurations.
     *
     * @param start [in] Start configuration of path
     * @param end [in] End configuration of path
     */
	rw::common::Ptr<QEdgeConstraintIncremental> instance(
        const rw::math::Q& start,
        const rw::math::Q& end) const;

    /**
     * @brief The start configuration of the path.
     */
    const rw::math::Q& getStart() const;

    /**
     * @brief The end configuration of the path.
     */
    const rw::math::Q& getEnd() const;

    /**
     * @brief Reset the object to use a different pair of start and end
     * configurations.
     */
    void reset(const rw::math::Q& start, const rw::math::Q& end);

    // Here we have the factory methods.

    /**
     * @brief Discrete path verification for a linearly interpolated path.
     *
     * Linearly interpolate from \b start to \b end configuration until the
     * distance between pairs of configurations is \b resolution when
     * measured by \b metric. Verify each configuration by \b constraint.
     *
     * The cost is defined as the distance (measured by \b metric) between
     * pairs of configurations currently verified by \b constraint.
     *
     * The metric must be well-behaved, i.e. linear.
     *
     * You can pass empty configurations as \b start and \b end to construct
     * an initial edge planner that you can instance() with better
     * configurations later.
     *
     * Start and end configurations for this initial planner are set to the
     * empty configuration.
     */
	static rw::common::Ptr<QEdgeConstraintIncremental> make(
		rw::common::Ptr<QConstraint> constraint,
		rw::common::Ptr<Metric<rw::math::Q> > metric,
        double resolution = 1);

    /**
     * @brief Default edge constraint for a configuration constraint and a
     * device.
     *
     * Start and end configurations are connected by a straight line in the
     * configuration space and are checked by a default collision checking
     * resolution.
     */
	static rw::common::Ptr<QEdgeConstraintIncremental> makeDefault(
		rw::common::Ptr<QConstraint> constraint,
		rw::common::Ptr<Device> device);

    /**
     * @brief A fixed edge constraint.
     *
     * The fixed edge constraint always returns \b value from inCollision().
     */
    static
		rw::common::Ptr<QEdgeConstraintIncremental> makeFixed(bool value);

    // We can implement a bunch of other instances, for example an instance
    // parameterized by an interpolator.

protected:
    /**
     * @brief Constructor provided for subclasses.
     *
     * @param start [in] Start configuration of path
     * @param end [in] End configuration of path
     */
    QEdgeConstraintIncremental(
        const rw::math::Q& start,
        const rw::math::Q& end);

    /**
     * @brief Subclass implementation of the inCollision() method.
     *
     * By default the method is implemented in terms of instance() and
     * inCollision().
     */
    virtual bool doInCollision(
        const rw::math::Q& start,
        const rw::math::Q& end) const;

    /**
     * @brief Subclass implementation of the inCollision() method.
     *
     * By default this method is implemented in terms of
     * inCollisionPartialCheck() and isFullyChecked().
     */
    virtual bool doInCollision();

    /**
     * @brief Subclass implementation of the inCollisionCost() method.
     */
    virtual double doInCollisionCost() const = 0;

    /**
     * @brief Subclass implementation of the inCollisionPartialCheck() method.
     *
     * By default this method is implemented in terms of inCollision().
     */
    virtual bool doInCollisionPartialCheck();

    /**
     * @brief Subclass implementation of the isFullyChecked() method.
     */
    virtual bool doIsFullyChecked() const = 0;

    /**
     * @brief Subclass implementation of the instance() method.
     */
	virtual rw::common::Ptr<QEdgeConstraintIncremental> doClone(
        const rw::math::Q& start,
        const rw::math::Q& end) const = 0;

    /**
     * @brief Subclass implementation of the reset() method.
     *
     * The start and end configurations will be reset before doReset() is
     * called, and therefore the start and end configurations are not passed
     * to doReset().
     */
    virtual void doReset() = 0;
};

%template (QEdgeConstraintIncrementalPtr) rw::common::Ptr<QEdgeConstraintIncremental>;

/**
 * @brief Interface for the sampling a configuration that solves an IK
 * problem.
 */
class QIKSampler
{
public:
    /**
     * @brief Sample a configuration that solves an IK problem for \b
     * target.
     *
     * If sampling fails, the sampler may return the empty configuration. If
     * empty() is true then the sampler has no more configurations.
     * Otherwise sample() may (or may not) succeed if called a second time.
     */
    rw::math::Q sample(const rw::math::Transform3D<double>& target);

    /**
     * @brief True if the sampler is known to contain no more
     * configurations.
     */
    bool empty() const;

    /**
     * @brief Destructor
     */
    virtual ~QIKSampler();

    /**
     * @brief An IK sampler based on an iterative IK solver.
     *
     * All solutions returned are checked to be within the bounds of the device.
     *
     * @param device [in] The device for which seeds are sampled.
     *
     * @param state [in] Fixed state with respect to which IK is solved.
     *
     * @param solver [in] Optional IK solver for \b device and \b state.
     *
     * @param seed [in] Optional sampler of seeds to feed the IK solver.
     *
     * @param maxAttempts [in] Optional number of seeds to feed the IK
     * solver. If \b maxAttempts is negative, a default value for \b
     * maxAttempts is chosen.
     */
	static rw::common::Ptr<QIKSampler> make(
		rw::common::Ptr<Device> device,
        const State& state,
		rw::common::Ptr<IterativeIK> solver = NULL,
		rw::common::Ptr<QSampler> seed = NULL,
        int maxAttempts = -1);

    /**
     * @brief An IK sampler filtered by a constraint.
     *
     * For each call of sample() up to \b maxAttempts configurations are
     * extracted from \b sampler and checked by \b constraint. The first
     * sample that satisfies the constraint is returned; if no such were
     * found the empty configuration is returned.
     *
     * If \b maxAttempts is negative, then \b sampler is sampled forever
     * until either the \b sampler is empty or a configuration satisfying \b
     * constraint is found.
     */
    static rw::common::Ptr<QIKSampler> makeConstrained(
	    rw::common::Ptr<QIKSampler> sampler,
		rw::common::Ptr<QConstraint> constraint,
        int maxAttempts = -1);

protected:
    /**
     * @brief Constructor
     */
    QIKSampler();

    /**
     * @brief Subclass implementation of the sample() method.
     */
    virtual rw::math::Q doSample(
        const rw::math::Transform3D<double>& target) = 0;

    /**
     * @brief Subclass implementation of the empty() method.
     *
     * By default the sampler is assumed to be sampling an infinite set of
     * configurations. IOW. the function returns false by default.
     */
    virtual bool doEmpty() const;
};

%template (QIKSamplerPtr) rw::common::Ptr<QIKSampler>;
%template (QIKSamplerCPtr) rw::common::Ptr<const QIKSampler>;

%nodefaultctor QToQPlanner;
class QToQPlanner: public PathPlanner<rw::math::Q, const rw::math::Q> {
public:

    %extend {
/*
        rw::common::Ptr<Path<rw::math::Q> > query(rw::math::Q from, rw::math::Q to, rw::common::Ptr<StopCriteria> stop){
            rw::common::Ptr<Path<rw::math::Q> > path = rw::common::ownedPtr(new PathQ());
            $self->rw::pathplanning::PathPlanner<rw::math::Q,const rw::math::Q>::query(from,to,*path,*stop);
            return path;
        }

        rw::common::Ptr<Path<rw::math::Q> > query(rw::math::Q from, rw::math::Q to, double time){
            rw::common::Ptr<Path<rw::math::Q> > path = rw::common::ownedPtr(new PathQ());
            $self->rw::pathplanning::PathPlanner<rw::math::Q,const rw::math::Q>::query(from,to,*path,time);
            return path;
        }

        rw::common::Ptr<Path<rw::math::Q> > query(rw::math::Q from, rw::math::Q to){
            rw::common::Ptr<Path<rw::math::Q> > path = rw::common::ownedPtr(new PathQ());
            $self->rw::pathplanning::PathPlanner<rw::math::Q,const rw::math::Q>::query(from,to,*path);
            return path;
        }

        PropertyMap& getProperties(){
            return $self->rw::pathplanning::PathPlanner<rw::math::Q,const rw::math::Q>::getProperties();
        }
*/
/*
        static rw::common::Ptr<QToQPlanner> makeRRT(rw::common::Ptr<CollisionDetector> cdect, rw::common::Ptr<Device> dev, const State& state){
            const rw::pathplanning::PlannerConstraint constraint = rw::pathplanning::PlannerConstraint::make(
                cdect.get(), dev, state);
            return rwlibs::pathplanners::RRTPlanner::makeQToQPlanner(constraint, dev);
        }

        static rw::common::Ptr<QToQPlanner> makeSBL(rw::common::Ptr<CollisionDetector> cdect, rw::common::Ptr<Device> dev, const State& state){
            rw::pathplanning::QConstraint::Ptr qconstraint = rw::pathplanning::QConstraint::make(cdect.get(), dev, state);
            return rwlibs::pathplanners::SBLPlanner::makeQToQPlanner(rwlibs::pathplanners::SBLSetup::make(qconstraint, rw::pathplanning::QEdgeConstraintIncremental::makeDefault(qconstraint, dev), dev));
        }
*/
    };
};

%template (QToQPlannerPtr) rw::common::Ptr<QToQPlanner>;

%nodefaultctor QToTPlanner;
class QToTPlanner 
{
public:
	//! @brief smart pointer type to this class
	typedef rw::common::Ptr<QToTPlanner> Ptr;

    %extend {

        rw::common::Ptr<Path<rw::math::Q> > query(rw::math::Q from, rw::math::Transform3D<double> to, rw::common::Ptr<StopCriteria> stop){
            rw::common::Ptr<Path<rw::math::Q> > path = rw::common::ownedPtr(new PathQ());
            $self->rw::pathplanning::PathPlanner<rw::math::Q,const rw::math::Transform3D<double> >::query(from,to,*path,*stop);
            return path;
        }

        rw::common::Ptr<Path<rw::math::Q> > query(rw::math::Q from, rw::math::Transform3D<double> to, double time){
            rw::common::Ptr<Path<rw::math::Q> > path = rw::common::ownedPtr(new PathQ());
            $self->rw::pathplanning::PathPlanner<rw::math::Q,const rw::math::Transform3D<double> >::query(from,to,*path,time);
            return path;
        }

        rw::common::Ptr<Path<rw::math::Q> > query(rw::math::Q from, rw::math::Transform3D<double> to){
            rw::common::Ptr<Path<rw::math::Q> > path = rw::common::ownedPtr(new PathQ());
            $self->rw::pathplanning::PathPlanner<rw::math::Q,const rw::math::Transform3D<double> >::query(from,to,*path);
            return path;
        }

        PropertyMap& getProperties(){
            return $self->rw::pathplanning::PathPlanner<rw::math::Q,const rw::math::Transform3D<double> >::getProperties();
        }
	}	
};

%template (QToTPlannerPtr) rw::common::Ptr<QToTPlanner>;

%nodefaultctor QToQSamplerPlanner;
/**
   @brief Sampled region planner interface.

   QToQSamplerPlanner plans a configuration space path from a start
   configuration to any configuration in the set represented by a sampler.
*/
class QToQSamplerPlanner : public PathPlanner<rw::math::Q, QSampler>
{
};

%template (QToQSamplerPlannerPtr) rw::common::Ptr<QToQSamplerPlanner>;

/**
   @brief Interface for the checking for collisions for work cell states.
*/
class StateConstraint
{
public:
	/**
	 * @brief Set the log to be used for writing debug info
	 *
	 * @param log [in] Log to which debug information is to be written
	 */
	virtual void setLog(rw::common::Ptr<Log> log);

    /**
     * @brief True if the work cell is considered to be in collision for the
     * work cell state \b state.
     */
    bool inCollision(const State& state) const;

    /**
     * Destructor
     */
    virtual ~StateConstraint();

    // Factory functions follow below.

    /**
     * @brief Map a collision detector to a state constraint.
     */
	static rw::common::Ptr<StateConstraint> make(
		rw::common::Ptr<CollisionDetector> detector);

    /**
     * @brief Combine a set of state constraints into a single state
     * constraint.
     */
	static rw::common::Ptr<StateConstraint> make(
		const std::vector<rw::common::Ptr<StateConstraint> >& constraints);

protected:
    /**
     * @brief Subclass implementation of the inCollision() method.
     */
    virtual bool doInCollision(const State& state) const = 0;

    /**
     * @brief Set a log.
     *
     * @param log [in] the log.
     */
	virtual void doSetLog(rw::common::Ptr<Log> log) = 0;

    /**
     * @brief Constructor
     */
    StateConstraint();
};

%template (StateConstraintPtr) rw::common::Ptr<StateConstraint>;
%template (StateConstraintCPtr) rw::common::Ptr<const StateConstraint>;
%template (StateConstraintPtrVector) std::vector<rw::common::Ptr<StateConstraint> >;
