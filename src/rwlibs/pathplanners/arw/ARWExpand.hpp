/*********************************************************************
 * RobWork Version 0.3
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

#ifndef RWLIBS_PATHPLANNERS_ARVEXPAND_HPP
#define RWLIBS_PATHPLANNERS_ARVEXPAND_HPP

/**
   @file ARWExpand.hpp
*/

#include <rw/common/Ptr.hpp>
#include <rw/math/Q.hpp>
#include <rw/models/Device.hpp>
#include <rw/pathplanning/PlannerConstraint.hpp>

namespace rwlibs { namespace pathplanners {

    /** @addtogroup pathplanning */
    /** @{*/

    class ARWExpand;

    //! A pointer to a ARWExpand.
    typedef rw::common::Ptr<ARWExpand> ARWExpandPtr;

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
        ARWExpandPtr duplicate(const rw::math::Q& start) const;

        /**
           @brief Destructor
        */
        virtual ~ARWExpand() {}

        /**
           @brief The current path of the random walk.
        */
        const std::vector<rw::math::Q>& getPath() const { return _path; }

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
        static ARWExpandPtr make(
            const rw::models::Device::QBox& bounds,
            const rw::pathplanning::PlannerConstraint& constraint,
            const rw::math::Q& minVariances = rw::math::Q(),
            int historySize = -1);

    protected:
        /**
           @brief Constructor
        */
        ARWExpand() {}

        /**
           @brief Subclass implementation of the expand() method.

           The doExpand() adds one or more nodes to \b _path if and only if the
           method returns true.
        */
        virtual bool doExpand() = 0;

        /**
           @brief Subclass implementation of the duplicate() method.
        */
        virtual ARWExpandPtr doDuplicate(const rw::math::Q& start) const = 0;

    private:
        ARWExpand(const ARWExpand&);
        ARWExpand& operator=(const ARWExpand&);

    protected:
        /**
           @brief The path of random walk.
        */
        std::vector<rw::math::Q> _path;
    };

    /* @} */
}} // end namespaces

#endif // end include guard
