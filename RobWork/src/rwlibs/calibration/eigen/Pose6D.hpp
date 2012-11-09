/*
* Pose6D.hpp
*
*  Created on: Oct 28, 2011
*      Author: bing
*/

#ifndef RWLIBS_CALIBRATION_POSE_HPP
#define RWLIBS_CALIBRATION_POSE_HPP

#include <rw/math.hpp>
#define EIGEN_TRANSFORM_PLUGIN "rwlibs/calibration/eigen/TransformAddons.hpp"

#include "RPY.hpp"

#include <Eigen/Core>

#include <iostream>

namespace rwlibs {
	namespace calibration {

		template<typename Scalar = double>
		class Pose6D: public Eigen::Matrix<Scalar, 6, 1> {
		public:
			Pose6D() : Eigen::Matrix<Scalar, 6, 1>() {

			}

			inline Pose6D(Scalar x, Scalar y, Scalar z, Scalar roll, Scalar pitch, Scalar yaw) {
				*this << x, y, z, roll, pitch, yaw;
			}

			explicit Pose6D(const Eigen::Affine3d& transform) {
				this->template head<3>() = transform.translation();
				RPY<Scalar> rpy(transform.linear());
				//this->template tail<3>() << rpy.roll(), rpy.pitch(), rpy.yaw();
				this->operator()(3) = rpy.roll();
				this->operator()(4) = rpy.pitch();
				this->operator()(5) = rpy.yaw();
			}

			template<typename OtherDerived>
			inline Pose6D(const Eigen::MatrixBase<OtherDerived>& other) {
				this->Eigen::Matrix<Scalar, 6, 1>::operator=(other);
			}

			inline Scalar& roll() {
				return this->operator()(3);
			}

			inline const Scalar& roll() const {
				return this->operator()(3);
			}

			inline Scalar& pitch() {
				return this->operator()(4);
			}

			inline const Scalar& pitch() const {
				return this->operator()(4);
			}

			inline Scalar& yaw() {
				return this->operator()(5);
			}

			inline const Scalar& yaw() const {
				return this->operator()(5);
			}

			inline Eigen::VectorBlock<Eigen::Matrix<Scalar, 6, 1>, 3> translation() {
				return this->template segment<3>(0);
			}

			inline const Eigen::VectorBlock<const Eigen::Matrix<Scalar, 6, 1>, 3> translation() const {
				return this->template segment<3>(0);
			}

			inline const RPY<Scalar> rotation() const {
				return RPY<Scalar>(this->roll(), this->pitch(), this->yaw());
			}

			inline Pose6D<Scalar> inverse() const {
				return Pose6D<Scalar>(toTransform().inverse());
			}

			inline Eigen::Affine3d toTransform() const {
				Eigen::Affine3d transform;
				transform.translation() = this->translation();
				transform.linear() = this->rotation().toRotationMatrix();
				return transform;
			}

			inline Eigen::Affine3d operator*(
				const Eigen::Affine3d& other) const {
					return this->toTransform() * other;
			}

			inline Pose6D<Scalar> operator*(
				const Pose6D<Scalar>& other) const {
					return Pose6D<Scalar>(this->toTransform() * other.toTransform());
			}

			inline operator Eigen::Affine3d() const {
				return this->toTransform();
			}

			inline static Pose6D<Scalar> Zero() {
				return Eigen::Matrix<Scalar, 6, 1>::Zero();
			}
		};

		typedef Pose6D<float> Pose6Df;
		typedef Pose6D<double> Pose6Dd;

	}
}

#endif /* RWLIBS_CALIBRATION_POSE_HPP */
