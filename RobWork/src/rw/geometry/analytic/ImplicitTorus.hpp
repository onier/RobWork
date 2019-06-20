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

#ifndef RW_GEOMETRY_ANALYTIC_IMPLICITTORUS_HPP_
#define RW_GEOMETRY_ANALYTIC_IMPLICITTORUS_HPP_

/**
 * @file ImplicitTorus.hpp
 *
 * \copydoc rw::geometry::ImplicitTorus
 */

#include "ImplicitSurface.hpp"

#include <rw/geometry/PlainTriMesh.hpp>

namespace rw {
namespace geometry {

    //! @addtogroup geometry

    //! @{
    /**
     * @brief Torus defined as an implicit surface.
     *
     * The torus is described as an implicit surface of the form:
     *
     * \f$ \left(x^T x + R^2 - r^2 \right)^2 - 4 R^2 x^T \begin{bmatrix} 1&0&0\\ 0&1&0 \\ 0&0&0 \end{bmatrix} x = 0\f$
     *
     * or equivalently:
     *
     * \f$ \left(x^T x - R^2 - r^2 \right)^2 - 4 R^2 (r^2 - {x_3}^2) = 0\f$
     *
     * where R is the distance from the center of the torus to the center of
     * the tube, r is the radius of the tube, and \f$ x \in \mathbb{R}^3\f$.
     *
     * \image html geometry/torus_circular.png "Standard circular torus with circular tube."
     *
     * Alternatively, a torus with an elliptic tube can be specified.
     * This surface has the more generic form:
     *
     * \f$ \left( x^T \begin{bmatrix} 1&0&0\\ 0&1&0 \\ 0&0&\frac{{r_1}^2}{{r_2}^2} \end{bmatrix} x + R^2 - {r_1}^2 \right)^2 - 4 R^2 x^T \begin{bmatrix} 1&0&0\\ 0&1&0 \\ 0&0&0 \end{bmatrix} x = 0\f$
     *
     * or equivalently:
     *
     * \f$ \left( x^T \begin{bmatrix} 1&0&0\\ 0&1&0 \\ 0&0&\frac{{r_1}^2}{{r_2}^2} \end{bmatrix} x - R^2 - {r_1}^2 \right)^2 - 4 R^2 \left({r_1}^2 - \frac{{r_1}^2}{{r_2}^2} {x_3}^2 \right) = 0\f$
     *
     * \image html geometry/torus_elliptic.png "Circular torus with elliptic tube."
     *
     * The elliptic torus with elliptic tube has the much more complex form:
     *
     * \f$ \left( ({R_2}^2+t){x_1}^2 + ({R_1}^2+t){x_2}^2 - ({R_1}^2+t)({R_2}^2+t) - 4 R_1 R_2 t \right)^2 - 4 t (R_2 {x_1}^2 + R_1 {x_2}^2 - (R_1+R_2)(R_1 R_2 + t))^2 = 0\f$
     *
     * where
     *
     * \f$ t = {r_1}^2 (1-\frac{{x_3}^2}{{r_2}^2}) \f$
     *
     * \image html geometry/torus_elliptic_elliptic.png "Elliptic torus with elliptic tube."
     *
     * Notice that many functions are not yet implemented for this last type of
     * elliptic torus. These functions might throw an exception.
     */
    class ImplicitTorus: public ImplicitSurface
    {
        public:
            //! @brief Smart pointer type for ImplicitTorus
            typedef rw::common::Ptr<ImplicitTorus> Ptr;

            //! @brief Smart pointer type for const ImplicitTorus
            typedef rw::common::Ptr<const ImplicitTorus> CPtr;

            /**
             * @brief A trimming region is defined using an ImplicitSurface.
             *
             * A point is only considered part of this surface, if all trimming conditions evaluate to a negative value.
             */
            typedef ImplicitSurface::CPtr TrimmingRegion;

            /**
             * @brief Construct circular torus.
             * @param R [in] distance from center of torus to center of tube.
             * @param r [in] radius of the tube.
             */
            ImplicitTorus(double R, double r);

            /**
             * @brief Construct elliptic torus.
             * @param R1 [in] distance from center of torus to center of tube in first direction.
             * @param R2 [in] distance from center of torus to center of tube in second direction.
             * @param r1 [in] radius of the tube in first direction.
             * @param r2 [in] radius of the tube in second direction.
             */
            ImplicitTorus(double R1, double R2, double r1, double r2);

            //! @brief Destructor.
            virtual ~ImplicitTorus();

            // From ImplicitSurface
            //! @copydoc ImplicitSurface::transform(const rw::math::Transform3D<>&) const
            ImplicitTorus::Ptr transform(const rw::math::Transform3D<>& T) const;

            //! @copydoc ImplicitSurface::transform(const rw::math::Vector3D<>&) const
            ImplicitTorus::Ptr transform(const rw::math::Vector3D<>& P) const;

            //! @copydoc ImplicitSurface::scale
            ImplicitTorus::Ptr scale(double factor) const;

            //! @copydoc ImplicitSurface::clone
            ImplicitTorus::Ptr clone() const;

            //! @copydoc ImplicitSurface::extremums
            virtual std::pair<double,double> extremums(const rw::math::Vector3D<>& direction) const;

            //! @copydoc ImplicitSurface::getTriMesh
            virtual rw::common::Ptr<TriMesh> getTriMesh(const std::vector<rw::math::Vector3D<> >& border = std::vector<rw::math::Vector3D<> >()) const;

            //! @copydoc ImplicitSurface::setDiscretizationResolution
            virtual void setDiscretizationResolution(double resolution) { _stepsPerRevolution = resolution; }

            //! @copydoc ImplicitSurface::equals
            virtual bool equals(const Surface& surface, double threshold) const;

            //! @copydoc ImplicitSurface::operator()(const rw::math::Vector3D<>&) const
            virtual double operator()(const rw::math::Vector3D<>& x) const;

            //! @copydoc ImplicitSurface::insideTrimmingRegion
            virtual bool insideTrimmingRegion(const rw::math::Vector3D<>& P) const;

            //! @copydoc ImplicitSurface::normal
            virtual rw::math::Vector3D<> normal(const rw::math::Vector3D<>& x) const;

            //! @copydoc ImplicitSurface::gradient
            virtual rw::math::Vector3D<> gradient(const rw::math::Vector3D<>& x) const;

            //! @copydoc ImplicitSurface::reuseTrimmingRegions
            virtual void reuseTrimmingRegions(ImplicitSurface::Ptr surface) const;

            /**
             * @brief Get the trimming conditions for the surface.
             * @return ImplicitSurface vector specifying the boundary of the surface. If surface is unbounded, the length of the vector is zero.
             */
            const std::vector<TrimmingRegion>& getTrimmingConditions() const { return _conditions; }

            /**
             * @brief Set the trimming conditions of this surface.
             * @param conditions [in] a vector of conditions.
             */
            void setTrimmingConditions(const std::vector<TrimmingRegion>& conditions) { _conditions = conditions; }

            /**
             * @brief Add a trimming condition to this surface.
             * @param condition [in] the condition to add.
             */
            void addTrimmingCondition(const TrimmingRegion& condition) { _conditions.push_back(condition); }

        private:
            ImplicitTorus(double R1, double R2, double r1, double r2, const rw::math::Transform3D<>& transform, const std::vector<TrimmingRegion>& conditions, double stepsPerRevolution);
            rw::common::Ptr<TriMesh> getTriMeshNormalForm(const std::vector<rw::math::Vector3D<> >& border, const rw::math::Transform3D<>& transform = rw::math::Transform3D<>::identity()) const;
            typedef enum Place {
                FRONT,
                BACK,
                BOTH
            } Place;
            void makeSurface(const std::vector<rw::math::Vector3D<> > fullPolygon, Place place, rw::geometry::PlainTriMeshN1D::Ptr mesh) const;

            // From ImplicitSurface
            inline ImplicitSurface::Ptr doTransformImplicitSurface(const rw::math::Transform3D<>& T) const { return transform(T); }
            inline ImplicitSurface::Ptr doTransformImplicitSurface(const rw::math::Vector3D<>& P) const { return transform(P); }
            inline ImplicitSurface::Ptr doScaleImplicitSurface(double factor) const { return scale(factor); }
            inline ImplicitSurface::Ptr doCloneImplicitSurface() const { return clone(); }

            const double _R1;
            const double _R2;
            const double _r1;
            const double _r2;

            std::vector<TrimmingRegion> _conditions;

            double _stepsPerRevolution;

            rw::math::Transform3D<> _transform;
            bool _isNormalForm;
    };
//! @}

} /* namespace geometry */
} /* namespace rw */

#endif /* RW_GEOMETRY_ANALYTIC_IMPLICITTORUS_HPP_ */
