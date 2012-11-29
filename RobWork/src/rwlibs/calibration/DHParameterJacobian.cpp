/*
* DHParameterJacobian.cpp
*
*  Created on: Nov 20, 2012
*      Author: bing
*/

#include "DHParameterJacobian.hpp"

namespace rwlibs {
	namespace calibration {
		DHParameterJacobian::DHParameterJacobian(DHParameterCalibration::Ptr calibration) : JacobianBase(calibration), _calibration(calibration) {
		}

		DHParameterJacobian::~DHParameterJacobian() {

		}

		Eigen::MatrixXd DHParameterJacobian::doComputeJacobian(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr targetFrame,
			const rw::kinematics::State& state) {
				const rw::models::Joint::Ptr joint = _calibration->getJoint();

				const Eigen::Affine3d tfmToPreLink = rw::kinematics::Kinematics::frameTframe(referenceFrame.get(), joint->getParent(state), state);
				const Eigen::Affine3d tfmLink = joint->getFixedTransform();
				const Eigen::Affine3d tfmToPostLink = tfmToPreLink * tfmLink;
				const Eigen::Affine3d tfmJoint = joint->getJointTransform(state);
				const Eigen::Affine3d tfmPostJoint = rw::kinematics::Kinematics::frameTframe(joint.get(), targetFrame.get(), state);
				const Eigen::Affine3d tfmToEnd = tfmToPostLink * tfmJoint * tfmPostJoint;

				const unsigned int columnCount = _calibration->getEnabledParameterCount();
				Eigen::MatrixXd jacobian(6, columnCount);
				int columnIndex = 0;
				if (_calibration->isParameterEnabled(DHParameterCalibration::PARAMETER_A)) {
					jacobian.block<3, 1>(0, columnIndex) = tfmToPostLink.linear().col(0);
					jacobian.block<3, 1>(3, columnIndex) = Eigen::Vector3d::Zero();
					columnIndex++;
				}
				if (_calibration->isParameterEnabled(DHParameterCalibration::PARAMETER_B)) {
					jacobian.block<3, 1>(0, columnIndex) = tfmToPreLink.linear().col(1);
					jacobian.block<3, 1>(3, columnIndex) = Eigen::Vector3d::Zero();
					columnIndex++;
				}
				if (_calibration->isParameterEnabled(DHParameterCalibration::PARAMETER_D)) {
					jacobian.block<3, 1>(0, columnIndex) = tfmToPreLink.linear().col(2);
					jacobian.block<3, 1>(3, columnIndex) = Eigen::Vector3d::Zero();
					columnIndex++;
				}
				if (_calibration->isParameterEnabled(DHParameterCalibration::PARAMETER_ALPHA)) {
					const Eigen::Vector3d xAxisToPost = tfmToPostLink.linear().col(0);
					const Eigen::Vector3d tlPostToEnd = tfmToEnd.translation() - tfmToPostLink.translation();
					jacobian.block<3, 1>(0, columnIndex) = xAxisToPost.cross(tlPostToEnd);
					jacobian.block<3, 1>(3, columnIndex) = xAxisToPost;
					columnIndex++;
				}
				if (_calibration->isParameterEnabled(DHParameterCalibration::PARAMETER_BETA)) {
					const Eigen::Vector3d yAxisToPre = tfmToPreLink.linear().col(1);
					const Eigen::Vector3d tlPreToEnd = tfmToEnd.translation() - tfmToPreLink.translation();
					jacobian.block<3, 1>(0, columnIndex) = yAxisToPre.cross(tlPreToEnd);
					jacobian.block<3, 1>(3, columnIndex) = yAxisToPre;
				}
				if (_calibration->isParameterEnabled(DHParameterCalibration::PARAMETER_THETA)) {
					const Eigen::Vector3d zAxisToPre = tfmToPreLink.linear().col(2);
					const Eigen::Vector3d tlPreToEnd = tfmToEnd.translation() - tfmToPreLink.translation();
					jacobian.block<3, 1>(0, columnIndex) = zAxisToPre.cross(tlPreToEnd);
					jacobian.block<3, 1>(3, columnIndex) = zAxisToPre;
				}

				return jacobian;
		}
	}
}
