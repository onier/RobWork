/*
 * Calibration.hpp
 *
 *  Created on: Aug 30, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_CALIBRATION_HPP_
#define RWLIBS_CALIBRATION_CALIBRATION_HPP_

#include <Eigen/Core>
#include <rw/kinematics.hpp>

namespace rwlibs {
namespace calibration {

class Calibration {
public:
	typedef rw::common::Ptr<Calibration> Ptr;

	virtual ~Calibration();

	virtual bool isEnabled() const;

	virtual void setEnabled(bool isEnabled);

	virtual bool isApplied() const;

	void apply();

	void revert();

	void correct(rw::kinematics::State& state);

	int getParameterCount() const;

	Eigen::MatrixXd compute(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame,
			const rw::kinematics::State& state);

	void step(const Eigen::VectorXd& step);

protected:
	Calibration();

	virtual void doApply() = 0;

	virtual void doRevert() = 0;

	virtual void doCorrect(rw::kinematics::State& state) = 0;

	virtual int doGetParameterCount() const = 0;

	virtual Eigen::MatrixXd doCompute(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame,
			const rw::kinematics::State& state) = 0;

	virtual void doStep(const Eigen::VectorXd& step) = 0;

private:
	bool _isEnabled;
	bool _isApplied;
};

}
}

#endif /* RWLIBS_CALIBRATION_CALIBRATION_HPP_ */
