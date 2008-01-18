//----------------------------------------------------------------------
// Below we have descriptions for the different namespaces. We basically have a
// namespace for each module.

/**
   @brief RobWork library.

   All classes and interfaces in this group only have dependencies on boost and
   STL. It is considered the core of the RobWork project.
 */
namespace rw {
	/**
	 * @brief Matrices, vectors, configurations, and more.
	 */
	namespace math {}
	/**
	 * @brief Various utilities and definitions of general use
	 */
	namespace common {}
	/**
	 * @brief Workcell and device models
	 */
	namespace models {}
	/**
	 * @brief Kinematic modelling
	 */
	namespace kinematics {}
	
    /**
     * @brief Inverse kinematics interfaces and iksolver classes
     */
	namespace invkin {}
	
	/**
	 * @brief Path-planning for devices
	 */	
	namespace pathplanning {}

	/**
	 * @brief Interfaces for collision checking and distance calculation.
	 */	
	namespace proximity {}
	
	/**
	 * @brief Loading and storing of CAD models
	 */	
	namespace geometry {}
	
	/**
	 *  @brief Interpolation and blending. This design is NOT STABLE.
	 */	
	namespace interpolator {}
	
	/**
	 * @brief Workcell loaders and other loaders
	 */	
	namespace loaders {}
	
	/**
	 * @brief Sensor interfaces
	 */	
	namespace sensor {}
	
	/**
	 * @brief Task descriptions
	 */
	namespace task {}
}

/**
  @brief Extension libraries for RobWork.

  Classes in this group can have specific dependencies on external libraries.
 */
namespace rwlibs {
	/**
	 * @brief Various algorithms
	 */
	namespace algorithms {} 

	/**
	 * @brief Device drivers
	 */
	namespace devices {} 

	/**
	 * @brief OpenGL drawing of workcells and geometries
	 */
	namespace drawable {} 

	/**
	 * @brief A collection of classes for performing device I/O
	 */
	namespace io {} 

	/**
	 * @brief A Lua interface to RobWork
	 */
	namespace lua {} 

	/**
	 * @brief A collection of OS specific include configurations
	 */
	namespace os {} 

	/**
	 * @brief A collection of pathoptimization algorihms
	 */
	namespace pathoptimization {} 

	/**
	 * @brief Path planners
	 */
	namespace pathplanners{} 

	/**
	 * @brief Proximity strategies
	 */
	namespace proximitystrategies {} 

	/**
	 * @brief Sensors
	 */
	namespace sensors {} 
}

/**

@defgroup rw rw
@{
    @copydoc rw

    @defgroup math math
    @{
        @copydoc rw::math
    @}

    @defgroup common common
    @{
        @copydoc rw::common
    @}

    @defgroup models models
    @{
        @copydoc rw::models
    @}

    @defgroup kinematics kinematics
    @{
        @copydoc rw::kinematics
    @}

    @defgroup invkin invkin
    @{
        @copydoc rw::invkin
    @}

    @defgroup pathplanning pathplanning
    @{
        @copydoc rw::pathplanning
    @}
    
    @defgroup pathoptimization pathoptimization
    @{
        @brief Path optimization algorithms
    @}

    @defgroup proximity proximity
    @{
        @copydoc rw::proximity
    @}

    @defgroup geometry geometry
    @{
        @copydoc rw::geometry
    @}

    @defgroup interpolator interpolator
    @{
        @copydoc rw::interpolator
    @}

    @defgroup loaders loaders
    @{
        @copydoc rw::loaders
    @}

    @defgroup sensor sensor
    @{
        @copydoc rw::sensor
    @}

    @defgroup task task
    @{
        @copydoc rw::task
    @}

    @defgroup 
    @{
        @brief Ignore this group. The group has been added to fix a glitch in
        the Doxygen output.
    @}
@}

@defgroup rwlibs rwlibs
@{
    @copydoc rwlibs

    @defgroup algorithms algorithms
    @{
        @copydoc rw::algorithms
    @}

    @defgroup proximitystrategies proximitystrategies
    @{
        @copydoc rw::proximitystrategies
    @}

    @defgroup devices devices
    @{
        @copydoc rw::devices
    @}

    @defgroup drawable drawable
    @{
        @copydoc rw::drawable
    @}

    @defgroup io io
    @{
        @copydoc rw::io
    @}

    @defgroup pathplanners pathplanners
    @{
        @copydoc rw::pathplanners
    @}

    @defgroup sensors sensors
    @{
        @copydoc rw::sensors
    @}

    @defgroup lua lua
    @{
        @copydoc rw::lua
    @}
	
    @defgroup os os
    @{
        @copydoc rw::os
    @}	

    @defgroup pathoptimization pathoptimization
    @{
        @copydoc rw::pathoptimization
    @}
    	
    @defgroup 
    @{
        @brief Ignore this group. The group has been added to fix a glitch in
        the Doxygen output.
    @}
@}

*/
