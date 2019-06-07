%module rw_pathplanners

%{
#include <rwlibs/swig/ScriptTypes.hpp>
#include <rw/common/Ptr.hpp>

using namespace rwlibs::swig;
using rw::math::Metric;
using rw::math::Q;
using rw::pathplanning::PathPlanner;
using rw::trajectory::Path;
%}

%import <rwlibs/swig/rw.i>

%pragma(java) jniclassimports=%{
import org.robwork.rw.*;
%}
%pragma(java) moduleimports=%{
import org.robwork.rw.*;
%}
%typemap(javaimports) SWIGTYPE %{
import org.robwork.rw.*;
%}

/**
   @brief ARWExpand expands a random walk in the configuration space by one
   step.
*/
class ARWExpand
{
public:
    /**
       @brief Expand the path by one step and return true if a new
       configuration was added to the path.

       The current path can be retrieved with ARWExpand::getPath().

       @return True iff a node was added to the end of the path.
    */
    bool expand();

    /**
       @brief Construct a new random walk with start node at \b start.
    */
	rw::common::Ptr<ARWExpand> duplicate(const rw::math::Q& start) const;

    /**
       @brief Destructor
    */
    virtual ~ARWExpand();

    /**
       @brief The current path of the random walk.
    */
	const Path<rw::math::Q>& getPath() const;

    /**
       @brief Constructor

       The expansion method computes the variance for the latest \b
       historySize elements of the path and performs one step sampled from a
       Gaussian distribution with the computed variances. Variances lower
       than \b minVariances are never used.

       If \b minVariances is empty, then a default value is chosen based on
       \b bounds.

       If \b historySize is negative, a default value is chosen.

       @param bounds [in] Configuration space bounds.

       @param constraint [in] Path planning constraint.

       @param minVariances [in] Minimum variances.

       @param historySize [in] Number of previous elements of the path to
       use for variance computation.
    */
	static rw::common::Ptr<ARWExpand> make(
        const std::pair<rw::math::Q, rw::math::Q>& bounds,
        const PlannerConstraint& constraint,
        const rw::math::Q& minVariances = rw::math::Q(),
        int historySize = -1);

protected:
    /**
       @brief Constructor
    */
    ARWExpand();

    /**
       @brief Subclass implementation of the expand() method.

       The doExpand() adds one or more nodes to \b _path if and only if the
       method returns true.
    */
    virtual bool doExpand() = 0;

    /**
       @brief Subclass implementation of the duplicate() method.
    */
	virtual rw::common::Ptr<ARWExpand> doDuplicate(const rw::math::Q& start) const = 0;

protected:
    /**
       @brief The path of random walk.
    */
	Path<rw::math::Q> _path;
};

//! @brief Smart pointer type for a ARWPlanner.
%template (ARWExpandPtr) rw::common::Ptr<ARWExpand>;

%nodefaultctor ARWPlanner;
/**
   @brief Adaptive Random Walk planners

   The ARW planners are based on the algorithm of: Stefano Carpin and
   Gianluigi Pillonetto, Motion Planning Using Adaptive Random Walks, IEEE
   Transactions on Robotics, Vol. 21, No. 1, 2005.

   @relates QToQPlanner
*/
class ARWPlanner
{
public:
    /**
       @brief ARW based point-to-point planner.

       The ARW planner expands its paths using instances of \b expand. If
       the end point of one of the paths is within a distance \b
       nearDistance from a goal node when measured with \b metric, then that
       connection is verified. If that connection is valid, the full path is
       returned.

       @param constraint [in] Path planning constraint.

       @param expand [in] ARW expansion strategy.

       @param metric [in] Distance to goal node measure.

       @param nearDistance [in] Threshold for distance to goal node.
    */
	static rw::common::Ptr<QToQPlanner> makeQToQPlanner(
        const PlannerConstraint& constraint,
		rw::common::Ptr<ARWExpand> expand,
		rw::common::Ptr<Metric<rw::math::Q> > metric,
        double nearDistance);

    /**
       @brief ARW based point-to-point planner.

       Based on \b device, a default distance metric and expansion strategy
       is chosen. A connection to a goal node is attempted if the distance
       is below \b nearDistance. A variance based expansion method is chosen
       with variances being calculated for the latest \b historySize
       samples.

       @param constraint [in] Path planning constraint.

       @param device [in] Device for which the path is planned.

       @param metric [in] Configuration space distance metric. If \b metric
       is NULL, a default metric for \b device is chosen.

       @param nearDistance [in] Try to connect to the goal if the distance
       to the goal measured by \b metric is below this threshold. If \b
       metric is null, a default value for \b nearDistance is chosen.

       @param historySize [in] Number of previous configurations on the path
       to include in computation of the next expand step. If \b historySize
       is negative, a default value for the parameter is chosen.
    */
	static rw::common::Ptr<QToQPlanner> makeQToQPlanner(
        const PlannerConstraint& constraint,
		rw::common::Ptr<Device> device,
		rw::common::Ptr<Metric<rw::math::Q> > metric = NULL,
        double nearDistance = -1,
        int historySize = -1);
};

//! @brief Smart pointer type for a ARWPlanner.
%template (ARWPlannerPtr) rw::common::Ptr<ARWPlanner>;
OWNEDPTR(ARWPlanner);

/**
 * @brief Implements a probabilistic roadmap (PRM) planner.
 *
 * The PRMPlanner is implemented freely after [1], and has a number of options:
 *
 * - Lazy Collision Checking: Using lazy collision checking as in [2], the
 * planner can be used for single as well as multiple queries.
 *
 * - Nearest Neighbor Search: The algorithm can either use a partial index
 * table [3] or a simple brute force method to do the nearest neighbor
 * search.
 *
 * - Shortest Path Algorithm: Using the Boost Graph Library, both A* and
 * Dijkstra's Algorithm may be used for finding the shortest path.
 *
 * As default the algorithm runs with lazy collision checking, brute force
 * neighbor search and with A* for shortest path search.
 *
 * As metric the PRMPlanner uses a WeightedEuclideanMetric for which it
 * estimates the weights such that it provides a worst-case estimate of the
 * Cartesian motion of the robots given a change in the configuration.
 *
 * Example of use
 * \code
 *      PRMPlanner* prm = new PRMPlanner(device, workcell, state, collisionDetector, resolution);
 *      prm->setCollisionCheckingStrategy(PRMPlanner::LAZY);
 *      prm->setNeighSearchStrategy(PRMPlanner::BRUTE_FORCE);
 *      prm->setShortestPathSearchStrategy(PRMPlanner::A_STAR);
 *      prm->buildRoadmap(1000);
 *      Path path;
 *      bool pathFound = prm->query(qstart, qgoal, path, maxtime);
 * \endcode
 *
 * [1]: Probabilistic Roadmaps for Path Planning in High-Dimensional
 *      Configuration Spaces, L.E. Kavraki, P. Svestka, J-C. Latombe, M.H.
 *      Overmars. IEEE Transactions on Robotics and Automation, Vol. 12, pages
 *      566-580, 1996
 *
 * [2]: Path Planning using Lazy PRM, R. Bohlin, L.E. Kavraki. Proceedings
 *      of the IEEE International Conference on Robotics and Automation, Vol. 1,
 *      pages 521-528, 2000
 *
 * [3]: On Delaying Collision Checking in PRM Planning - Application to
 *      Multi-Robot Coordination, G. Sanchez, J.C. Latombe. The International
 *      Journal of Robotics Research, Vol. 21, No. 1, pages 5-26, 2002
 */
class PRMPlanner: public QToQPlanner
{
public:
    /**
     * @brief Constructs PRMPlanner
     *
     * Constructs a PRMPlanner with a given setup. This method does NOT build the roadmap.
     * The method estimates the movements of the robot to construct a weighted metric as explained
     * in the general introduction.
     *
     * @param device [in] Device to plan for
     * @param state [in] State giving the setup of the workcell
     * @param collisionDetector [in] CollisionDetector to use
     *
     * @param resolution [in] Cartesian distance the robot is allowed to move
     * between collision checks.
     */
    PRMPlanner(
        Device* device,
        const State& state,
        CollisionDetector* collisionDetector,
        double resolution);

    /**
       @brief Constructs PRMPlanner

       @param constraint [in] Collision constraint
       @param sampler [in] Configuration space sampler
       @param resolution [in] Collision checking resolution
       @param device [in] Device characteristics
       @param state [in] State of rest of the workcell
    */
    PRMPlanner(
    	rw::common::Ptr<QConstraint> constraint,
		rw::common::Ptr<QSampler> sampler,
        double resolution,
        const Device& device,
        const State& state);

    /**
     * @brief Destructor
     */
    virtual ~PRMPlanner();

    /**
     * @brief Build the roadmap with the setup specified
     * @param nodecount [in] Number of nodes to insert
     */
    void buildRoadmap(size_t nodecount);


    /**
     * @brief Sets the desired average number of neighbors. Default value is 20.
     * @param n [in] Desired average number of neighbors
     */
    void setNeighborCount(size_t n);

    /**
     * @brief Enumeration for selecting the node neighbor search strategy
     */
    enum NeighborSearchStrategy {
        BRUTE_FORCE = 0, /*!< Run through all node and look a which a sufficient close. */
        PARTIAL_INDEX_TABLE, /*!< Use a partial index table to make an more efficient lookup. */
        KDTREE /*!< Use KD tree for neighbor search */
    };

    /**
     * @brief Sets up the nearest neighbor search strategy
     *
     * @param neighborSearchStrategy [in] The nearest neighbor search strategy
     */
    void setNeighSearchStrategy(NeighborSearchStrategy neighborSearchStrategy);

    /**
     * @brief Sets up the number of dimensions for the partial index table
     *
     * This setting only applies when using the PARTIAL_INDEX_TABLE strategy for nearest
     * neighbor search.
     *
     * \b dimensions should be within [1; _device->getDOF()]. The optimal
     * value of \b dimensions is a tradeoff between memory usage and time.
     * Selecting a value too high compared to the number of nodes in the roadmap
     * may introduce an increase in time due to additional bookkeeping.
     *
     * The default value is set to 4, which is found suitable for most devices
     * with 6 or 7 degrees of freedom.
     *
     * @param dimensions [in] Number of dimensions, which should be
     */
    void setPartialIndexTableDimensions(size_t dimensions);

    /**
     * @brief Enumeration for selecting the collision checking strategy
     */
    enum CollisionCheckingStrategy {
        LAZY = 0, /*!< Perform a Lazy collision checking (no checking on construction).*/
        NODECHECK, /*!< Only check node on construction, leave edges unchecked. */
        FULL /*!<Perform a full check of both nodes and edges. */
    };

    /**
     * @brief Sets up the collision checking strategy
     *
     * Note: Do not call this after the buildRoadmap as it may result in paths with collisions
     * @param collisionCheckingStrategy [in] The collision checking strategy
     */
    void setCollisionCheckingStrategy(CollisionCheckingStrategy collisionCheckingStrategy);

    /**
     * @brief Enumeration for selecing the shortest path search strategy
     */
    enum ShortestPathSearchStrategy {
        A_STAR = 0, /*!< Use A* to search for shortest path. */
        DIJKSTRA /*!< Use Dijkstra to search for shortest path. */
    };

    /**
     * @brief Sets up the shortest path search strategy
     *
     * Generally A* is the fastest algorithm, but given a small roadmap Dijkstra may
     * perform better.
     *
     * @param shortestPathSearchStrategy [in] shortestPathSearchStrategy
     */
    void setShortestPathSearchStrategy(ShortestPathSearchStrategy shortestPathSearchStrategy);

    /**
     * @brief Sets the max time of A* before terminating and calling dijkstra
     *
     * The A* implementation in the boost graph library has a reported bug, which on
     * some platforms in rare occasions may cause it to loop infinitely. If A* uses
     * more than this specified time it will break off and call dijkstra instead.
     *
     * Default value for this timeout is 1second.
     *
     * @brief timeout [in] Timeout time.
     */
    void setAStarTimeOutTime(double timeout);

    /**
     * @brief print timing stats from previous run.
     */
    void printTimeStats();
};

//! @brief Smart pointer type for a PRMPlanner.
%template (PRMPlannerPtr) rw::common::Ptr<PRMPlanner>;
OWNEDPTR(PRMPlanner);

%nodefaultctor RRTPlanner;
//! @brief RRT based planners
class RRTPlanner
{
public:
    //! The type of RRT planner to construct.
    enum PlannerType {
        /**
           @brief Simple non-greedy, bidirectional RRT.

           See BasicPlanner(), page 109 of James J. Kuffner, "Autonomous
           Agensts for Real-Time Animation", 1999.
        */
        RRTBasic,

        /**
           @brief RRT-Connect planner.

           See James J. Kuffner and Steven M. LaValle, "RRT-Connect: An
           Efficient Approach to Single-Query Path Planning", ICRA, 2000.
        */
        RRTConnect,

        /**
           @brief Bidirectional RRT.

           The algorithm of the planner is in the style of
           RDT_BALANCED_BIDIRECTIONAL(), page 195 of Steven M. Lavalle,
           "Planning Algorithms", 2006, except this planner is the non-balanced
           version.
        */
        RRTBidirectional,

        /**
           @brief Balanced, bidirectional RRT.

           The algorithm of the planner is in the style of
           RDT_BALANCED_BIDIRECTIONAL(), page 195 of Steven M. Lavalle,
           "Planning Algorithms", 2006.
        */
        RRTBalancedBidirectional
    };

    /**
       @brief RRT based point-to-point planner.

       @param constraint [in] Constraint for configurations and edges.

       @param sampler [in] Sampler of the configuration space.

       @param metric [in] Metric for nearest neighbor search.

       @param extend [in] Distance measured by \b metric by which to extend
       the tree towards an attractor configuration.

       @param type [in] The particular variation the RRT planner algorithm.
    */
	static rw::common::Ptr<QToQPlanner> makeQToQPlanner(
        const PlannerConstraint& constraint,
		rw::common::Ptr<QSampler> sampler,
		rw::common::Ptr<Metric<rw::math::Q> > metric,
        double extend,
        PlannerType type = RRTBalancedBidirectional);

    /**
       @brief RRT based point-to-point planner.

       Default configuration space sampling strategy
       (rw::pathplanning::QSampler) and distance metrics (rw:math::QMetric)
       are chosen based on \b device.

       @param constraint [in] Constraint for configurations and edges.

       @param device [in] Device for which the path is planned.

       @param type [in] The particular variation the RRT planner algorithm.
    */
	static rw::common::Ptr<QToQPlanner> makeQToQPlanner(
        const PlannerConstraint& constraint,
		rw::common::Ptr<Device> device,
        PlannerType type = RRTBalancedBidirectional);
};

//! @brief Smart pointer type for a RRTPlanner.
%template (RRTPlannerPtr) rw::common::Ptr<RRTPlanner>;
OWNEDPTR(RRTPlanner);

/**
   @brief Interface for sampling a configuration in the vicinity of some
   other configuration.

   SBLExpand is a primitive for planners in the SBL family. The primitive
   takes a configuration \b q as parameter and returns another configuration
   somewhere in the vicinity of \b q.

   Different implementations can have different policies with respect to
   what constraints are satisfied by the configurations returned.
*/
class SBLExpand
{
public:
    /**
       @brief A configuration sampled from the vicinity of \b q.

       Implementation dependant, the sampler may return the empty
       configuration if no configurations can be sampled near \b q.
    */
    rw::math::Q expand(const rw::math::Q& q);

    /**
       @brief A configuration space in the shape of a box.

       The box is given by a lower and upper corner.
    */
    typedef std::pair<rw::math::Q, rw::math::Q> QBox;

    /**
       @brief Expansion within the overlap of an inner and outer box.

       Given a configuration \b q, the expand() method returns a configuration
       sampled uniformly at random from the intersection between
		\code
		    q + inner
		\endcode
		           and
		\code
		    outer
		\endcode

       Given a \b device, you typically use \b device.getBounds() as the box
       for the outer configuration space.

       If the overlap between the boxes is empty, expand() returns the empty
       configuration.
    */
    static rw::common::Ptr<SBLExpand> makeUniformBox(
        const QBox& outer,
        const QBox& inner);

    /**
       @brief Expansion within a scaled down box of the configuration space.

       Given a configuration \b q, the expand() method samples a
       configuration uniformly at random from the intersection between
		\code
		    q + inner
		\endcode
		           and
		\code
		    outer
		\endcode
       where \b inner equals \b outer scaled by a factor of \b ratio and
       centered at origo.

       This is a form of expansion you will use in a standard implementation
       of an SBL planner.

       \b ratio must be positive.

       If \b outer is non-empty, the expand() method will always return a
       non-empty configuration.
    */
    static rw::common::Ptr<SBLExpand> makeUniformBox(
        const QBox& outer,
        double ratio);

    /**
       @brief Sample within a box of decreasing size until a collision free
       configuration is found.

       The inner box shrinks in size as 1, 1/2, 1/3, ...

       This form of expansion is typical for SBL planners.

       The inner and outer box are specified as explained for
       makeUniformBox().
    */
    static rw::common::Ptr<SBLExpand> makeShrinkingUniformBox(
    	rw::common::Ptr<QConstraint> constraint,
        const QBox& outer,
        const QBox& inner);

    /**
       @brief Sample within a box of shrinking size until a collision free
       configuration is found.

       The inner box shrinks in size as 1, 1/2, 1/3, ...

       This form of expansion is typical for SBL planners.

       The inner and outer box are specified as explained for
       makeUniformBox().
    */
    static rw::common::Ptr<SBLExpand> makeShrinkingUniformBox(
    	rw::common::Ptr<QConstraint> constraint,
        const QBox& outer,
        double ratio);

    /**
       @brief Sample within a box of shrinking size until a collision free
       configuration is found.

       The size of the inner box depends on the Jacobian of the current
       configuration. The radius for the i'th dimension of the inner box is

       R_i = min(angle_max / angle_vel, disp_max / disp_vel)

       where angle_vel is the magnitude of the angular velocity and disp_vel
       is the magnitude of the translational velocity.

       If \b jacobian is NULL, a default device Jacobian is chosen based on
       \b device.

       If \b angle_max or \b disp_max is negative, a default value for the
       variable is chosen.

       The inner box shrinks in size as 1, 1/2, 1/3, ...
    */
    static rw::common::Ptr<SBLExpand> makeShrinkingUniformJacobianBox(
    	rw::common::Ptr<QConstraint> constraint,
		rw::common::Ptr<Device> device,
        const State& state,
        rw::common::Ptr<JacobianCalculator> jacobian,
        double angle_max = -1,
        double disp_max = -1);

    /**
       @brief Destructor
    */
    virtual ~SBLExpand() {}

protected:
    /**
       @brief Constructor
    */
    SBLExpand() {}

    /**
       @brief Subclass implementation of the expand() method.
    */
    virtual rw::math::Q doExpand(const rw::math::Q& q) = 0;

private:
    SBLExpand(const SBLExpand&);
    SBLExpand& operator=(const SBLExpand&);
};

//! @brief Smart pointer type for a SBLExpand.
%template (SBLExpandPtr) rw::common::Ptr<SBLExpand>;
OWNEDPTR(SBLExpand);

/**
   @brief Common parameters for SBL based planners.

   All versions of the SBL planner base verify configurations and paths in
   the configuration space using a PlannerConstraint object.

   In addition, parameters can given to define how expansion around a node
   of the tree should be done and under what circumstances the two trees
   should be connected.

   A SBLSetup object stores pointers to the shared objects, but can be
   copied and assigned freely.
*/
class SBLSetup
{
public:
    /**
       @brief Constructor

       The SBL planner for this setup performs brute force search for the
       nearest neighbor of the other tree, and attempts to connect the trees
       if the distance to the neighbor is below a given threshold.

       @param constraint [in] Planning constraint.

       @param edgeConstraint [in] Planning constraint for edges.

       @param expansion [in] Expansion strategy for insertion of new nodes.
       The nodes returned by the expansion strategy must be collision free
       or empty. If an empty configuration is returned, the planner tries to
       expand somewhere else.

       @param metric [in] Distance metric for nearest neighbor searching.

       @param connectRadius [in] Attempt connection of the trees if the
       distance to the nearest neighbor is below this threshold.
    */
    static
    SBLSetup make(
		rw::common::Ptr<QConstraint> constraint,
		rw::common::Ptr<QEdgeConstraintIncremental> edgeConstraint,
        rw::common::Ptr<SBLExpand> expansion,
		rw::common::Ptr<Metric<rw::math::Q> > metric,
        double connectRadius);

    /**
       @brief Constructor

       Simple default expansion and tree connection strategies are chosed
       based on the device for which the planning is done.

       The planner expands uniformly at random with a maximum stepsize of \b
       expandRadius relative to the diameter of the configuration space. The
       step size and the diameter is measured by the infinity metric.

       The planner connect a newly created node to the nearest node of the
       other tree if the distance to the other node (measured by the
       infinity metric and relative to the diameter of the configuration
       space) is less than \b connectRadius.

       If \b expandRadius or \b connectRadius is negative, a default value
       is chosen.

       @param constraint [in] Planning constraint.

       @param edgeConstraint [in] Planning constraint for edges.

       @param device [in] Device for which planning is done.

       @param expandRadius [in] Node expansion radius.

       @param connectRadius [in] Neighbor connection radius.
    */
    static
    SBLSetup make(
		rw::common::Ptr<QConstraint> constraint,
		rw::common::Ptr<QEdgeConstraintIncremental> edgeConstraint,
		rw::common::Ptr<Device> device,
        double expandRadius = -1,
        double connectRadius = -1);

private:
    SBLSetup(const SBLOptions& options);

public:
    //! @brief Internal options to use.
    SBLOptions options;
    friend class SBLInternal;
};

//! @brief A SBL planner constraint.
class SBLPlannerConstraint {
public:
	/**
	 * @brief Constructor for a planner constrinct.
	 * @param qconstraint [in] a constraint giving the valid (collision free) configurations.
	 * @param edgeconstraint [in] a constraint for checking the edges in-between valid configurations.
	 */
	SBLPlannerConstraint(rw::common::Ptr<QConstraint> qconstraint, 
		rw::common::Ptr<QEdgeConstraintIncremental> edgeconstraint):
	_qconstraint(qconstraint),
	_edgeConstraint(edgeconstraint);

	/**
	 * @brief Get the part that checks for valid configurations.
	 * @return a reference to the constraint.
	 */
	const QConstraint& getQConstraint() const;

	/**
	 * @brief Get the part that checks edges in-between valid configurations.
	 * @return a reference to the edge constraint.
	 */
	const QEdgeConstraintIncremental& getEdgeConstraint() const;
};

/**
   @brief SBL planner setup.

   SBLOptions is the value stored in SBLSetup.

   SBLOptions is a seperate file so that we can keep SBLSetup as abstract as
   possible.

   SBLOptions is used by SBLInternal and is for internal use only.
*/
class SBLOptions
{
public:
	/**
	 * @brief Construct a new set of options for the internal algorithms.
	 * @param constraint [in] a constraint on the valid configurations.
	 * @param edgeConstraint [in] a constraint on the edges between valid configurations.
	 * @param expansion [in] the policy for how to sample new configurations in the vicinity.
	 * @param metric [in] the distance metric for nearest neighbor searching.
	 * @param connectRadius [in] connect trees if the distance to the nearest neighbor is below this threshold.
	 */
    SBLOptions(
		rw::common::Ptr<QConstraint>& constraint,
		rw::common::Ptr<QEdgeConstraintIncremental>& edgeConstraint,
		rw::common::Ptr<SBLExpand> expansion,
		rw::common::Ptr<Metric<rw::math::Q> > metric,
        double connectRadius);

    //! @brief The constraint that determined if a path or configuration is valid (collision free) or not.
    SBLPlannerConstraint constraint;
    //! @brief The expand policy used to sample new configurations in the vicinity.
    rw::common::Ptr<SBLExpand> expansion;
    //! @brief the distance metric for nearest neighbor searching.
	rw::common::Ptr<Metric<rw::math::Q> > metric;
	//! @brief Attempt connection of the trees if the distance to the nearest neighbor is below this threshold.
    double connectRadius;

    //! @brief Policy for choosing a node in the vicinity of a given node, \b n.
    enum NearNodeSelection {
        UniformSelect,  //!< take a random node.
        UniformFromCell,//!< take a random node within the cell where node \b n lies.
        NearestFromCell,//!< take the nearest node from the cell where node \b n lies.
        NearestNode     //!< search for the nearest node (default)
    };

    //! @brief Policy for selecting a tree.
    enum TreeSelection {
        UniformTree, //!< randomly select one of the two trees (default)
        WeightedTree,//!< choose the tree randomly, but weighted according to the size of the tree.
        SmallestTree,//!< choose the smallest tree.
        LargestTree  //!< choose the largest tree.
    };

    //! @brief Policy for how often to connect trees.
    enum ConnectFrequency {
        ConnectAlways,//!< always connect (default)
        ConnectAtReset//!< connect only at reset.
    };

    //! @brief (default is 20).
    int resetCount;
    //! @brief (default is 25).
    int rootSampleInterval;
    //! @brief (default is 10).
    double nodesPerCell;
    //! @brief (default is NearestNode).
    NearNodeSelection nearNodeSelection;
    //! @brief (default is UniformTree).
    TreeSelection treeSelection;
    //! @brief (default is ConnectAlways).
    ConnectFrequency connectAt;
};

%nodefaultctor SBLPlanner;
/**
   @brief SBL based planners.
*/
class SBLPlanner
{
public:
    /**
       @brief An SBL based sampled region planner.

       @param setup [in] Setup for the planner.
    */
	static rw::common::Ptr<QToQSamplerPlanner> makeQToQSamplerPlanner(const SBLSetup& setup);

    /**
       @brief An SBL based point-to-point planner.

       @param setup [in] Setup for the planner.
    */
	static rw::common::Ptr<QToQPlanner> makeQToQPlanner(const SBLSetup& setup);

    /**
       @brief An SBL based point-to-tool-position planner.

       @param setup [in] Setup for the planner.
       @param ikSampler [in] Sampler of IK solutions for the target transform.
    */
	static rw::common::Ptr<QToTPlanner> makeQToTPlanner(
        const SBLSetup& setup,
		rw::common::Ptr<QIKSampler> ikSampler);
};

//! @brief Smart pointer type for a SBLPlanner.
%template (SBLPlannerPtr) rw::common::Ptr<SBLPlanner>;
OWNEDPTR(SBLPlanner);

%nodefaultctor Z3Planner;
/**
   @brief Z3 based planners

   See "The Z3-Method for Fast Path Planning in Dynamic Environments", Boris
   Baginski, 1996.

   @relates QToQPlanner
*/
class Z3Planner
{
public:
    /**
       @brief Z3 based point-to-point planner.

       @param sampler [in] Sampler of the configuration space.

       @param localPlanner [in] Local planner for connecting the configurations.

       @param nodeCnt [in] Number of supporting configurations to insert.
       If \b nodeCnt is negative, a default value is chosen.

       @param repeatCnt [in] Number of times to repeat the attempt. If \b
       repeatCnt is negative (the default), the attempts are repeated until
       the stop criteria returns true.
    */
	static rw::common::Ptr<QToQPlanner> makeQToQPlanner(
		rw::common::Ptr<QSampler> sampler,
		rw::common::Ptr<QToQPlanner> localPlanner,
        int nodeCnt = -1,
        int repeatCnt = -1);

    /**
       @brief Z3 based point-to-point planner.

       A default configuration space sampler (rw::pathplanning::QSampler)
       and local planning is chosen for \b device using \b constraint for
       collision checking.

       @param constraint [in] Constraint for configurations and edges.

       @param device [in] Device for which the path is planned.
    */
	static rw::common::Ptr<QToQPlanner> makeQToQPlanner(
        const PlannerConstraint& constraint,
		rw::common::Ptr<Device> device);

    /**
       @brief Sliding local planner.

       This is a variation of the sliding local planner described in the Z3
       paper.

       This is the default local planner used for instantiation of the Z3
       based planners.

       @param constraint [in] Path planning constraint.

       @param directionSampler [in] Sampler of direction vectors in the
       configuration space.

       @param boundsConstraint [in] Constraint checking for the bounds of
       the configuration space.

       @param metric [in] Configuration space distance measure.

       @param extend [in] The length of each sliding step as measured by \b
       metric.

       @param slideImprovement [in] The minimum decrease in distance to the
       goal that should be acheived for every valid slide step. If \b
       slideImprovement is negative, a default value for \b slideImprovement
       is chosen based on the value of \b extend.
    */
	static rw::common::Ptr<QToQPlanner> makeSlidingQToQPlanner(
        const PlannerConstraint& constraint,
		rw::common::Ptr<QSampler> directionSampler,
		rw::common::Ptr<QConstraint> boundsConstraint,
		rw::common::Ptr<Metric<rw::math::Q> > metric,
        double extend,
        double slideImprovement = -1);

    /**
       @brief Sliding local planner.

       A default direction sampler and bounds checker is chosen for \b
       device.

       @param constraint [in] Path planning constraint.

       @param device [in] Device for which the planning is done.

       @param metric [in] Configuration space distance measure. If no metric
       is given, a default metric for \b device is chosen. In this case \b
       extend and \b slideImprovement should be negative, and default values
       for these will be chosen.

       @param extend [in] The length of each sliding step as measured by \b
       metric.

       @param slideImprovement [in] The minimum decrease in distance to the
       goal that should be acheived for every valid slide step. If \b
       slideImprovement is negative, a default value for \b slideImprovement
       is chosen based on the value of \b extend.
    */
    static rw::common::Ptr<QToQPlanner> makeSlidingQToQPlanner(
        const PlannerConstraint& constraint,
		rw::common::Ptr<Device> device,
		rw::common::Ptr<Metric<rw::math::Q> > metric = 0,
        double extend = -1,
        double slideImprovement = -1);
};

//! @brief Smart pointer type for a Z3Planner.
%template (Z3PlannerPtr) rw::common::Ptr<Z3Planner>;
OWNEDPTR(Z3Planner);

%inline %{

	rw::common::Ptr<QToTPlanner> makeToNearestRRT(rw::common::Ptr<CollisionDetector> cdect, rw::common::Ptr<Device> dev)
    { 
		rw::kinematics::State state = dev->getStateStructure()->getDefaultState();
      
        const rw::pathplanning::PlannerConstraint constraint = rw::pathplanning::PlannerConstraint::make(
            cdect.get(), dev, state);
      
        rw::common::Ptr<QToQPlanner> planner = rwlibs::pathplanners::RRTPlanner::makeQToQPlanner(constraint, dev);

		// creaty ikmeta solver for the planer
        rw::invkin::JacobianIKSolver::Ptr iksolver = rw::common::ownedPtr( new rw::invkin::JacobianIKSolver(dev,state) );
       	rw::invkin::IKMetaSolver::Ptr metasolver = rw::common::ownedPtr( new rw::invkin::IKMetaSolver(iksolver, dev, cdect) );
		rw::common::Ptr<rw::pathplanning::QIKSampler> sampler = rw::pathplanning::QIKSampler::make(dev,state,metasolver);
        
		rw::math::QMetric::Ptr metric = rw::pathplanning::PlannerUtil::timeMetric(*dev);

		return rw::pathplanning::QToTPlanner::makeToNearest(planner, sampler, metric, 10); 
    }
%}