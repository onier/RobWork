/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
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

#ifndef rw_collision_ProximityCommon_HPP_
#define rw_collision_ProximityCommon_HPP_

#include <rw/kinematics/Frame.hpp>

#include <list>
#include <vector>
#include <string>

namespace rw { namespace proximity {

	/** @addtogroup proximity */
	/*@{*/

    /**
     * @brief A pair of frames
     */
    typedef std::pair<kinematics::Frame*, kinematics::Frame*> FramePair;

    /**
       @brief A set of frame pairs.
    */
    typedef std::set<FramePair> FramePairSet;

    /**
     * @brief A list of frame pairs
     */
    typedef std::list<FramePair> FramePairList;

    /**
     * @brief A pair of frame names
     */
    typedef std::pair<std::string, std::string> ProximityPair;

    /**
     * @brief A list of pairs for with ProximityPairs
     */
    typedef std::vector<ProximityPair> ProximityPairList;

	/*@}*/
}} // end namespaces

#endif // end include guard
