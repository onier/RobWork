/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute, 
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


#ifndef RW_MATH_Pose3D_HPP
#define RW_MATH_Pose3D_HPP

/**
 * @file Pose3D.hpp
 */

#include "EAA.hpp"
#include "Vector3D.hpp"
#include "Transform3D.hpp"

namespace rw { namespace math {

    /** @addtogroup math */
    /*@{*/

    /**
     * @brief A Pose3D @f$ \mathbf{x}\in \mathbb{R}^6 @f$ describes a position
     * and orientation in 3-dimensions.
     *
     * @f$ {\mathbf{x}} = \left[
     *  \begin{array}{c}
     *  x \\
     *  y \\
     *  z \\
     *  \theta k_x \\
     *  \theta k_y \\
     *  \theta k_z
     *  \end{array}
     *  \right]
     *  @f$
     *
     * where @f$ (x,y,z)@f$ is the 3d position and @f$ (\theta k_x, \theta k_y,
     * \theta k_z)@f$ describes the orientation in equal angle axis (EAA)
     * format.
     */
    template<class T = double>
    class Pose3D {
    private:
        Vector3D<T> _position;
        EAA<T> _orientation;

    public:
        /**
         * @brief Creates a Pose3D from 6 parameters. 3 defining the
         * position and 3 defining the EAA orientation.
         * @param x [in] The position in the @f$ x @f$ axis
         * @param y [in] The position in the @f$ y @f$ axis
         * @param z [in] The position in the @f$ z @f$ axis
         * @param kx [in] @f$ \theta k_x @f$
         * @param ky [in] @f$ \theta k_y @f$
         * @param kz [in] @f$ \theta k_z @f$
         */
        Pose3D(T x, T y, T z, T kx, T ky, T kz) :
            _position(x,y,z),
            _orientation(kx,ky,kz)
        {}

        /**
         * @brief Creates a Pose3D from a Vector3D and a EAA
         * @param v3d [in] Vector3D describing the 3D position of the Pose3D
         * @param eaa [in] EAA describing the rotational component of the Pose3D.
         */
        Pose3D(const Vector3D<T> &v3d, const EAA<T> &eaa):
            _position(v3d),
            _orientation(eaa)
        {}

        /**
         * @brief Creates a Pose3D from a Transform3D
         *
         * @param t3d [in] A Transform3D
         */
        explicit Pose3D(const Transform3D<T> &t3d):
            _position(t3d.P()),
            _orientation(t3d.R())
        {}

        /**
         * @brief Returns the \f$i\f$'th element in the pose.
         *
         * \f$i\in\{0,1,2\} \f$ corresponds to \f$\{x,y,z\}\f$ respectively.
         * \f$i\in\{3,4,5\}\f$ corresponds to the equivalent angle axis.
         *
         * @param i [in] index to return
         * @return the \f$i\f$'th index of the pose.
         */
        T get(size_t i) const {
            assert(i < 6);
            if (i < 3)
                return _position(i);
            else
                return _orientation.axis()(i-3)*_orientation.angle();
        }

        EAA<T> getEAA() const{
        	return _orientation;
        }

        /**
         * @brief Returns the \f$i\f$'th element in the pose.
         *
         * \f$i\in\{0,1,2\} \f$ corresponds to \f$\{x,y,z\}\f$ respectively.
         * \f$i\in\{3,4,5\}\f$ corresponds to the equivalent angle axis.
         *
         * @param i [in] index to return
         *
         * @return the \f$i\f$'th index of the pose.
         */
        T operator()(size_t i) const {
            assert(i < 6);
            if (i < 3)
                return _position(i);
            else
                return _orientation.axis()(i-3)*_orientation.angle();
        }

        /**
         * @brief Returns the \f$i\f$'th element in the pose.
         *
         * \f$i\in\{0,1,2\} \f$ corresponds to \f$\{x,y,z\}\f$ respectively.
         * \f$i\in\{3,4,5\}\f$ corresponds to the equivalent angle axis.
         *
         * @param i [in] index to return
         *
         * @return the \f$i\f$'th index of the pose.
         */
        T operator[](size_t i) const {
            assert(i < 6);
            if (i < 3)
                return _position(i);
            else
                return _orientation.axis()(i-3)*_orientation.angle();
        }

        /**
         * @brief Converts the Pose3D into the corresponding Transform3D
         * @return the corresponding Transform3D
         */
        const Transform3D<T> toTransform3D() {
            return Transform3D<T>(_position, _orientation);
        }

        /**
         * @brief Casts Pose3D<T> to Pose3D<Q>
         * @param pose [in] Pose3D with type T
         * @return Pose3D with type Q
         */
        template<class Q>
        friend const Pose3D<Q> cast(const Pose3D<T>& pose) {
            return Pose3D<Q>(
                static_cast<Q>(pose.get(0)),
                static_cast<Q>(pose.get(1)),
                static_cast<Q>(pose.get(2)),
                static_cast<Q>(pose.get(3)),
                static_cast<Q>(pose.get(4)),
                static_cast<Q>(pose.get(5)));
        }
    };

    /**
     * @brief Streaming operator.
     *
     * @relates Q
     */
    template <class T>
    std::ostream& operator<<(std::ostream& out, const Pose3D<T>& v)
    {
        return out
            << "Pose3D {"<< v(0)<< ", "<< v(1)<< ", " << v(2)
            << ", " << v(3) << ", " << v(4) << ", " << v(5) << "}";
    }

    /*@}*/
}} // end namespaces

#endif // end include guard
