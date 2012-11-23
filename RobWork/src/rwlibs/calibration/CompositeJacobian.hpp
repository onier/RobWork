/*
 * CompositeJacobian.hpp
 *
 *  Created on: Nov 20, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_COMPOSITEJACOBIAN_HPP_
#define RWLIBS_CALIBRATION_COMPOSITEJACOBIAN_HPP_

#include "Jacobian.hpp"

namespace rwlibs {
namespace calibration {
/** @addtogroup calibration */
/*@{*/

/**
 * @brief CompositeJacobian combines several calibrations.
 *
 * T can be Calibration or subclasses thereof.
 */
template<class T>
class CompositeJacobian: public Jacobian {
public:
	typedef rw::common::Ptr<CompositeJacobian> Ptr;

	/**
	* @brief Constructor.
	*/
	CompositeJacobian();

	/**
	* @brief Destructor.
	*/
	virtual ~CompositeJacobian();

	/**
	* @brief Returns a reference to a vector with pointers to the Jacobian(s) in the CompositeJacobian.
	* @return std::vector with pointers to Jacobian(s)
	*/
	const std::vector<rw::common::Ptr<T> >& getJacobians() const;

    /**
    * @brief Add calibration.
    * @param[in] calibration Pointer to calibration to be added.
    */
	void add(rw::common::Ptr<T> calibration);
	
	/**
	 * @copydoc Jacobian::isEnabled()
	 */
	virtual bool isEnabled() const;

	/**
	 * @copydoc Jacobian::setEnabled()
	 */
	virtual void setEnabled(bool isEnabled);

	virtual int getParameterCount() const;

	virtual bool isParameterEnabled(int parameterIndex);

	virtual void setParameterEnabled(int parameterIndex, bool isEnabled);

	virtual Eigen::MatrixXd computeJacobian(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr targetFrame,
			const rw::kinematics::State& state);

	virtual void takeStep(const Eigen::VectorXd& step);

private:

private:
	bool _isEnabled;
	std::vector<rw::common::Ptr<T> > _jacobians;
};

template<class T>
CompositeJacobian<T>::CompositeJacobian() {

}

template<class T>
CompositeJacobian<T>::~CompositeJacobian() {

}

template<class T>
const std::vector<rw::common::Ptr<T> >& CompositeJacobian<T>::getJacobians() const {
	return _jacobians;
}

template<class T>
void CompositeJacobian<T>::add(rw::common::Ptr<T> jacobian) {
	_jacobians.push_back(jacobian);
}

template<class T>
bool CompositeJacobian<T>::isEnabled() const {
	return _isEnabled;
}

template<class T>
void CompositeJacobian<T>::setEnabled(bool isEnabled) {
	_isEnabled = isEnabled;
}

template<class T>
int CompositeJacobian<T>::getParameterCount() const {
	if (!isEnabled())
		return 0;

	int parameterCount = 0;
	for (typename std::vector<rw::common::Ptr<T> >::const_iterator it = _jacobians.begin(); it != _jacobians.end(); ++it) {
		const rw::common::Ptr<T> parametrization = (*it);
		parameterCount += parametrization->getParameterCount();
	}
	return parameterCount;
}

template<class T>
bool CompositeJacobian<T>::isParameterEnabled(int parameterIndex) {
	RW_ASSERT(parameterIndex < getParameterCount());

	rw::common::Ptr<T> parametrization;
	int parameterIndexMin = 0, parameterIndexMax = 0;
	for (typename std::vector<rw::common::Ptr<T> >::const_iterator it = _jacobians.begin(); it != _jacobians.end(); ++it) {
		parametrization = (*it);
		const int parameterCount = parametrization->getParameterCount();
		parameterIndexMax += parameterCount;
		if (parameterIndex >= parameterIndexMin && parameterIndex < parameterIndexMax)
			break;
		parameterIndexMin = parameterIndexMax;
	}

	RW_ASSERT(parameterIndexMin != parameterIndexMax);

	return parametrization->isParameterEnabled(parameterIndex - parameterIndexMin);
}

template<class T>
void CompositeJacobian<T>::setParameterEnabled(int parameterIndex, bool isEnabled) {
	RW_ASSERT(parameterIndex < getParameterCount());

	rw::common::Ptr<T> parametrization;
	int parameterIndexMin = 0, parameterIndexMax = 0;
	for (typename std::vector<rw::common::Ptr<T> >::const_iterator it = _jacobians.begin(); it != _jacobians.end(); ++it) {
		parametrization = (*it);
		const int parameterCount = parametrization->getParameterCount();
		parameterIndexMax += parameterCount;
		if (parameterIndex >= parameterIndexMin && parameterIndex < parameterIndexMax)
			break;
		parameterIndexMin = parameterIndexMax;
	}

	RW_ASSERT(parameterIndexMin != parameterIndexMax);
	
	parametrization->setParameterEnabled(parameterIndex - parameterIndexMin, isEnabled);
}

template<class T>
Eigen::MatrixXd CompositeJacobian<T>::computeJacobian(rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr targetFrame,
		const rw::kinematics::State& state) {
	RW_ASSERT(getParameterCount() != 0);

	int parameterIndex = 0;
	Eigen::MatrixXd jacobian(6, getParameterCount());
	for (typename std::vector<rw::common::Ptr<T> >::iterator it = _jacobians.begin(); it != _jacobians.end(); ++it) {
		rw::common::Ptr<T> parametrization = (*it);
		const int parameterCount = parametrization->getParameterCount();
		if (parameterCount > 0) {
			jacobian.block(0, parameterIndex, 6, parameterCount) = parametrization->computeJacobian(referenceFrame, targetFrame, state);
			parameterIndex += parameterCount;
		}
	}

	return jacobian;
}

template<class T>
void CompositeJacobian<T>::takeStep(const Eigen::VectorXd& step) {
	RW_ASSERT(getParameterCount() != 0);

	unsigned int parameterIndex = 0;
	for (typename std::vector<rw::common::Ptr<T> >::iterator it = _jacobians.begin(); it != _jacobians.end(); ++it) {
		rw::common::Ptr<T> parametrization = (*it);
		unsigned int parameterCount = parametrization->getParameterCount();
		if (parameterCount > 0) {
			parametrization->takeStep(step.segment(parameterIndex, parameterCount));
			parameterIndex += parameterCount;
		}
	}
}

/*@}*/
}
}

#endif /* RWLIBS_CALIBRATION_COMPOSITEJACOBIAN_HPP_ */
