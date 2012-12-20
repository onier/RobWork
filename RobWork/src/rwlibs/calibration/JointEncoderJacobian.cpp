/*
* JointEncoderJacobian.cpp
*
*  Created on: Dec 20, 2012
*      Author: bing
*/

#include "JointEncoderJacobian.hpp"

namespace rwlibs {
	namespace calibration {
		JointEncoderJacobian::JointEncoderJacobian(JointEncoderCalibration::Ptr calibration) : JacobianBase(calibration), _calibration(calibration), _device(calibration->getDevice()), _joint(calibration->getJoint()) {
			// Find joint number.
			const std::vector<rw::models::Joint*> joints = _device->getJoints();
			_jointIndex = std::find(joints.begin(), joints.end(), _joint.get()) - joints.begin();
		}

		JointEncoderJacobian::~JointEncoderJacobian() {

		}

		Eigen::MatrixXd JointEncoderJacobian::doComputeJacobian(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr targetFrame,
			const rw::kinematics::State& state) {
				const CalibrationParameterSet parameterSet = _calibration->getParameterSet();

				// Prepare transformations.
				const Eigen::Affine3d tfmToPostJoint = rw::kinematics::Kinematics::frameTframe(referenceFrame.get(), _joint.get(), state);
				const Eigen::Affine3d tfmPostJoint = rw::kinematics::Kinematics::frameTframe(_joint.get(), targetFrame.get(), state);
				const Eigen::Affine3d tfmToEnd = tfmToPostJoint * tfmPostJoint;
				const Eigen::Vector3d posToEnd = tfmToEnd.translation() - tfmToPostJoint.translation();
				const Eigen::Vector3d jointAxis = tfmToPostJoint.linear().col(2);

				// Get joint value.
				const rw::math::Q q = _device->getQ(state);
				RW_ASSERT(_jointIndex < q.size());
				const double qi = q[_jointIndex];

				const int columnCount = getColumnCount();
				Eigen::MatrixXd jacobian(6, columnCount);
				int columnIndex = 0;
				if (parameterSet(JointEncoderCalibration::PARAMETER_TAU).isEnabled()) {
					jacobian.block<3, 1>(0, columnIndex) = -sin(qi) * jointAxis.cross(posToEnd);
					jacobian.block<3, 1>(3, columnIndex) = -sin(qi) * jointAxis;
					columnIndex++;
				}
				if (parameterSet(JointEncoderCalibration::PARAMETER_SIGMA).isEnabled()) {
					jacobian.block<3, 1>(0, columnIndex) = -cos(qi) * jointAxis.cross(posToEnd);
					jacobian.block<3, 1>(3, columnIndex) = -cos(qi) * jointAxis;
					columnIndex++;
				}
				
				RW_ASSERT(columnIndex == columnCount);

				return jacobian;
		}
	}
}
