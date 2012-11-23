/*
* DHParameterJacobian.cpp
*
*  Created on: Nov 20, 2012
*      Author: bing
*/

#include "DHParameterJacobian.hpp"

namespace rwlibs {
	namespace calibration {

	DHParameterJacobian::DHParameterJacobian(DHParameterCalibration::Ptr calibration) : JacobianBase(4), _calibration(calibration) {
		// Warn if b/beta representation is not used for close to parallel joints.
		const rw::models::Joint::Ptr joint = calibration->getJoint();
		rw::models::DHParameterSet dhParameterSet = *rw::models::DHParameterSet::get(joint.get());
		if (abs(dhParameterSet.alpha()) < (10 * rw::math::Deg2Rad) && !dhParameterSet.isParallel())
			RW_WARN("Joint \"" << joint->getName() << "\": DH alpha parameter close to zero. Singularities might occur, consider using b/beta parameters instead of d/theta.");
	}

	DHParameterJacobian::~DHParameterJacobian() {

	}

	Eigen::MatrixXd DHParameterJacobian::doComputeJacobian(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr targetFrame,
		const rw::kinematics::State& state) {
			RW_ASSERT(!referenceFrame.isNull());
			RW_ASSERT(!targetFrame.isNull());
			RW_ASSERT(_calibration->isApplied());

			const rw::models::Joint::Ptr joint = _calibration->getJoint();
			const Eigen::Affine3d tfmToPreLink = rw::kinematics::Kinematics::frameTframe(referenceFrame.get(), joint->getParent(state), state);
			const Eigen::Affine3d tfmLink = joint->getFixedTransform();
			const Eigen::Affine3d tfmToPostLink = tfmToPreLink * tfmLink;
			const Eigen::Affine3d tfmJoint = joint->getJointTransform(state);
			const Eigen::Affine3d tfmPostJoint = rw::kinematics::Kinematics::frameTframe(joint.get(), targetFrame.get(), state);
			const Eigen::Affine3d tfmToEnd = tfmToPostLink * tfmJoint * tfmPostJoint;

			const bool isParallel = rw::models::DHParameterSet::get(joint.get())->isParallel();

			const unsigned int columnCount = getParameterCount();
			Eigen::MatrixXd jacobian(6, columnCount);
			int columnIndex = 0;
			// a
			if (isParameterEnabled(PARAMETER_A)) {
				jacobian.block<3, 1>(0, columnIndex) = tfmToPostLink.linear().col(0);
				jacobian.block<3, 1>(3, columnIndex) = Eigen::Vector3d::Zero();
				columnIndex++;
			}
			// b/d
			if (isParameterEnabled(PARAMETER_B_D)) {
				jacobian.block<3, 1>(0, columnIndex) = tfmToPreLink.linear().col(isParallel ? 1 : 2);
				jacobian.block<3, 1>(3, columnIndex) = Eigen::Vector3d::Zero();
				columnIndex++;
			}
			// alpha
			if (isParameterEnabled(PARAMETER_ALPHA)) {
				Eigen::Vector3d xAxisToPost = tfmToPostLink.linear().col(0);
				Eigen::Vector3d tlPostToEnd = tfmToEnd.translation() - tfmToPostLink.translation();
				jacobian.block<3, 1>(0, columnIndex) = xAxisToPost.cross(tlPostToEnd);
				jacobian.block<3, 1>(3, columnIndex) = xAxisToPost;
				columnIndex++;
			}
			// beta/theta
			if (isParameterEnabled(PARAMETER_BETA_THETA)) {
				Eigen::Vector3d yzAxisToPre = tfmToPreLink.linear().col(isParallel ? 1 : 2);
				Eigen::Vector3d tlPreToEnd = tfmToEnd.translation() - tfmToPreLink.translation();
				jacobian.block<3, 1>(0, columnIndex) = yzAxisToPre.cross(tlPreToEnd);
				jacobian.block<3, 1>(3, columnIndex) = yzAxisToPre;
			}

			return jacobian;
	}

	void DHParameterJacobian::doTakeStep(const Eigen::VectorXd& step) {
		RW_ASSERT(step.rows() == 4);
		RW_ASSERT(_calibration->isApplied());

		_calibration->revert();

		const Eigen::Vector4d correction = _calibration->getCorrection();
		_calibration->setCorrection(correction + step);

		_calibration->apply();
	}

	}
}
