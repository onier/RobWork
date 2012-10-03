/*
 * CompositeCalibration.hpp
 *
 *  Created on: Sep 13, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_COMPOSITECALIBRATION_HPP_
#define RWLIBS_CALIBRATION_COMPOSITECALIBRATION_HPP_

#include "Calibration.hpp"

namespace rwlibs {
namespace calibration {
/** @addtogroup calibration */
/*@{*/

/**
 * @brief CompositeCalibration combines several calibrations.
 *
 * T can be Calibration or subclasses thereof.
 */
template<class T>
class CompositeCalibration: public Calibration {
public:
	typedef rw::common::Ptr<CompositeCalibration> Ptr;
	typedef typename std::vector<rw::common::Ptr<T> >::iterator iterator;

	/**
	* @brief Constructor.
	*/
	CompositeCalibration();

	/**
	* @brief Destructor.
	*/
	virtual ~CompositeCalibration();

	/**
	* @brief Returns a reference to a vector with pointers to the Calibration(s) in the CompositeCalibration.
	* @return std::vector with pointers to Calibration(s)
	*/
	const std::vector<rw::common::Ptr<T> >& getCalibrations() const;

    /**
    * @brief Add calibration.
    * @param[in] calibration Pointer to calibration to be added.
    */
	void add(rw::common::Ptr<T> calibration);

private:
	virtual void doApply();

	virtual void doRevert();

	virtual void doCorrect(rw::kinematics::State& state);

	virtual int doGetParameterCount() const;

	virtual Eigen::MatrixXd doComputeJacobian(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr targetFrame,
			const rw::kinematics::State& state);

	virtual void doTakeStep(const Eigen::VectorXd& step);

private:
	std::vector<rw::common::Ptr<T> > _calibrations;
};

template<class T>
CompositeCalibration<T>::CompositeCalibration() {

}

template<class T>
CompositeCalibration<T>::~CompositeCalibration() {

}

template<class T>
const std::vector<rw::common::Ptr<T> >& CompositeCalibration<T>::getCalibrations() const {
	return _calibrations;
}

template<class T>
void CompositeCalibration<T>::add(rw::common::Ptr<T> calibration) {
	_calibrations.push_back(calibration);
}

template<class T>
void CompositeCalibration<T>::doApply() {
	for (typename std::vector<rw::common::Ptr<T> >::iterator it = _calibrations.begin(); it != _calibrations.end(); ++it) {
		rw::common::Ptr<T> calibration = (*it);
		if (!calibration->isApplied() && !calibration->isLocked())
			calibration->apply();
	}
}

template<class T>
void CompositeCalibration<T>::doRevert() {
	for (typename std::vector<rw::common::Ptr<T> >::iterator it = _calibrations.begin(); it != _calibrations.end(); ++it) {
		rw::common::Ptr<T> calibration = (*it);
		if (calibration->isApplied() && !calibration->isLocked())
			calibration->revert();
	}
}

template<class T>
void CompositeCalibration<T>::doCorrect(rw::kinematics::State& state) {
	for (typename std::vector<rw::common::Ptr<T> >::iterator it = _calibrations.begin(); it != _calibrations.end(); ++it) {
		rw::common::Ptr<T> calibration = (*it);
		if (calibration->isApplied() && !calibration->isLocked())
			calibration->correct(state);
	}
}

template<class T>
int CompositeCalibration<T>::doGetParameterCount() const {
	int parameterCount = 0;
	for (typename std::vector<rw::common::Ptr<T> >::const_iterator it = _calibrations.begin(); it != _calibrations.end(); ++it)
		parameterCount += (*it)->getParameterCount();
	return parameterCount;
}

template<class T>
Eigen::MatrixXd CompositeCalibration<T>::doComputeJacobian(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr targetFrame,
		const rw::kinematics::State& state) {
	unsigned int parameterNo = 0;
	Eigen::MatrixXd jacobian(6, getParameterCount());
	for (typename std::vector<rw::common::Ptr<T> >::iterator it = _calibrations.begin(); it != _calibrations.end(); ++it) {
		rw::common::Ptr<T> calibration = (*it);
		unsigned int parameterCount = calibration->getParameterCount();
		if (parameterCount > 0) {
			jacobian.block(0, parameterNo, 6, parameterCount) = calibration->computeJacobian(referenceFrame, targetFrame, state);
			parameterNo += parameterCount;
		}
	}
	return jacobian;
}

template<class T>
void CompositeCalibration<T>::doTakeStep(const Eigen::VectorXd& step) {
	unsigned int parameterNo = 0;
	for (typename std::vector<rw::common::Ptr<T> >::iterator it = _calibrations.begin(); it != _calibrations.end(); ++it) {
		rw::common::Ptr<T> calibration = (*it);
		unsigned int parameterCount = calibration->getParameterCount();
		if (parameterCount > 0) {
			calibration->takeStep(step.segment(parameterNo, parameterCount));
			parameterNo += parameterCount;
		}
	}
}

/*@}*/
}
}

#endif /* RWLIBS_CALIBRATION_COMPOSITECALIBRATION_HPP_ */
