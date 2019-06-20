/******************************************************************************
 * Copyright 2019 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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
 ******************************************************************************/

#ifndef RW_GEOMETRY_ANALYTIC_IMPLICITBREP_HPP_
#define RW_GEOMETRY_ANALYTIC_IMPLICITBREP_HPP_

/**
 * @file ImplicitBREP.hpp
 *
 * \copydoc rw::geometry::ImplicitBREP
 */

#include "BREP.hpp"
#include "ImplicitSurface.hpp"
#include "ParametricCurve.hpp"

namespace rw {
namespace geometry {

    class ImplicitShell;

    //! @addtogroup geometry

    //! @{
    /**
     * @brief Type of BREP where all surfaces are of type ImplicitSurface,
     * and edges are of type ParametricCurve.
     */
    class ImplicitBREP: public BREP
    {
        public:
            //! @brief Smart pointer type to ImplicitBREP
            typedef rw::common::Ptr<ImplicitBREP> Ptr;

            //! @brief Smart pointer type to const ImplicitBREP
            typedef rw::common::Ptr<const ImplicitBREP> CPtr;

            //! @brief Constructor.
            ImplicitBREP();

            //! @brief Destructor.
            virtual ~ImplicitBREP();

            //! @copydoc BREP::getType
            virtual GeometryType getType() const;

            //! @copydoc BREP::getSurface
            virtual const ImplicitSurface& getSurface(std::size_t surfaceIndex) const;

            //! @copydoc BREP::getCurve
            virtual const ParametricCurve& getCurve(std::size_t curveIndex) const;

            //! @copydoc BREP::getCurve
            virtual void scale(double factor);

            //! @copydoc BREP::clone
            ImplicitBREP::Ptr clone() const;

            //! @copydoc BREP::shellProxy
            rw::common::Ptr<const ImplicitShell> shellProxy() const;

            //! @copydoc BREP::getCurves
            std::vector<rw::common::Ptr<ParametricCurve> > getCurves(std::size_t loopIdx) const;

            //! @brief Convenience type for a set of curves in a BREP.
            class CommonParametricCurveSet {
            public:
                //! @brief Smart pointer type to CommonParametricCurveSet
                typedef rw::common::Ptr<const CommonParametricCurveSet> CPtr;

                //! @brief Constructor.
                CommonParametricCurveSet() {}

                //! @brief Destructor.
                virtual ~CommonParametricCurveSet() {}

                //! @copydoc BREP::CommonCurveSet::size
                virtual std::size_t size() const = 0;

                //! @copydoc BREP::CommonCurveSet::curve
                virtual const ParametricCurve& curve(std::size_t index) const = 0;

                //! @copydoc BREP::CommonCurveSet::surfaceLeft
                virtual const ImplicitSurface& surfaceLeft(std::size_t index) const = 0;

                //! @copydoc BREP::CommonCurveSet::surfaceRight
                virtual const ImplicitSurface& surfaceRight(std::size_t index) const = 0;
            };

            //! @copydoc BREP::getCommonCurves
            CommonParametricCurveSet::CPtr getCommonCurves(const std::set<std::size_t>& faces) const;

            /**
             * @brief Add a ParametricCurve to the BREP.
             *
             * Notice that the curve has direction. It is expected to have limits such that it starts in vertex \b v1 and end in \b v2.
             *
             * @param curve [in] curve to add.
             * @param v1 [in] the start vertex.
             * @param v2 [in] the end vertex.
             */
            void addEdge(const ParametricCurve& curve, std::size_t v1, std::size_t v2);

            /**
             * @brief Attach an ImplicitSurface to a face of the BREP.
             * @param surface [in] surface to add.
             * @param loop [in] the loop index for the loop to attach surface to.
             */
            void setFace(const ImplicitSurface& surface, std::size_t loop);

        protected:
            /**
             * @brief Remove specific curve.
             * @param curveIndex [in] the index of the curve to remove.
             */
            virtual void doRemoveCurve(std::size_t curveIndex);

        private:
            class CommonParametricCurveSetImpl;
            virtual rw::common::Ptr<const Shell> doShellProxyBREP() const;
            virtual BREP::Ptr doClone() const { return clone(); }

            // Geometry
            std::vector<ParametricCurve::CPtr> _curves;
            std::vector<ImplicitSurface::CPtr> _surfaces;
    };
//! @}

} /* namespace geometry */
} /* namespace rw */

#endif /* RW_GEOMETRY_ANALYTIC_IMPLICITBREP_HPP_ */
