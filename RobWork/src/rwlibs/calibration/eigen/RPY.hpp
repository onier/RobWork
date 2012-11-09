/*
* RPY.hpp
*
*  Created on: Oct 23, 2011
*      Author: bing
*/

#ifndef RWLIBS_CALIBRATION_RPY_HPP
#define RWLIBS_CALIBRATION_RPY_HPP

#define _USE_MATH_DEFINES
#include <math.h>
#include <Eigen/Geometry>

#include <rw/common.hpp>

// http://stackoverflow.com/questions/652155/invalid-use-of-incomplete-type
namespace rwlibs {
	namespace calibration {

		template<typename _Scalar>
		class RPY;
	}
}

namespace Eigen {
	namespace internal {
		template<typename _Scalar> struct traits<rwlibs::calibration::RPY<_Scalar> > {
			typedef _Scalar Scalar;
		};
	}
}

namespace rwlibs {
	namespace calibration {

		template<typename _Scalar>
		class RPY: public Eigen::RotationBase<RPY<_Scalar>, 3> {
		public:
			RPY() {
			}

			RPY(const _Scalar& roll, const _Scalar& pitch, const _Scalar& yaw) {
				roll_ = roll;
				pitch_ = pitch;
				yaw_ = yaw;
			}

			template<typename Derived>
			RPY(const Eigen::MatrixBase<Derived>& matrix, _Scalar epsilon = 1e-5) {
				if (matrix.rows() != 3 || (matrix.cols() != 1 && matrix.cols() != 3))
					RW_THROW("RPY must be constructed from a 3x1 vector or 3x3 matrix.");

				if (matrix.cols() == 1) {
					roll_ = matrix(0,0);
					pitch_ = matrix(1,0);
					yaw_ = matrix(2,0);
				} else if (matrix.cols() == 3) {
					Eigen::Vector3d rpy = matrix.eulerAngles(2, 1, 0);
					roll_ = rpy(2);
					pitch_ = rpy(1);
					yaw_ = rpy(0);
				}
			}

			inline const _Scalar& roll() const {
				return roll_;
			}

			inline const _Scalar& pitch() const {
				return pitch_;
			}

			inline const _Scalar& yaw() const {
				return yaw_;
			}

			inline Eigen::Matrix<_Scalar, 3, 1> toVector() const {
				return Eigen::Matrix<_Scalar, 3, 1>(roll_, pitch_, yaw_);
			}

			inline Eigen::Matrix<_Scalar, 3, 3> toRotationMatrix() const {
				return Eigen::Matrix<_Scalar, 3, 3>(
					Eigen::AngleAxis<_Scalar>(roll_, Eigen::Matrix<_Scalar, 3, 1>::UnitZ()) * Eigen::AngleAxis<_Scalar>(pitch_, Eigen::Matrix<_Scalar, 3, 1>::UnitY())
					* Eigen::AngleAxis<_Scalar>(yaw_, Eigen::Matrix<_Scalar, 3, 1>::UnitX()));
			}

		private:
			_Scalar roll_;
			_Scalar pitch_;
			_Scalar yaw_;
		};

		typedef RPY<float> RPYf;
		typedef RPY<double> RPYd;

	}
}

#endif /* RWLIBS_CALIBRATION_RPY_HPP */
