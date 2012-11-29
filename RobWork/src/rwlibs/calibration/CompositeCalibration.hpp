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

	/**
	* @brief Constructor.
	*/
	CompositeCalibration();

	/**
	* @brief Constructor.
	*/
	CompositeCalibration(const std::vector<rw::common::Ptr<T> >& calibrations);

	/**
	* @brief Destructor.
	*/
	virtual ~CompositeCalibration();
	
	/**
	 * @return Number of calibrations.
	 */
	int getCalibrationCount() const;

	/**
	* @brief Returns a reference to a vector with pointers to the Calibration(s) in the CompositeCalibration.
	* @return std::vector with pointers to Calibration(s)
	*/
	const std::vector<rw::common::Ptr<T> >& getCalibrations() const;

	void addCalibration(rw::common::Ptr<T>& calibration);
	
	virtual bool isEnabled() const;

	virtual void setEnabled(bool isEnabled);

	virtual double getParameterValue(int parameterIndex) const;
	
	virtual void setParameterValue(int parameterIndex, double value);
	
	virtual int getParameterCount() const;
	
	virtual int getEnabledParameterCount() const;
	
	virtual bool isParameterEnabled(int parameterIndex) const;
	
	virtual void setParameterEnabled(int parameterIndex, bool isEnabled);
	
	/**
	 * @copydoc Calibration::isApplied()
	 */
	virtual bool isApplied() const;
	
	/**
	 * @copydoc Calibration::apply()
	 */
	virtual void apply();
	
	/**
	 * @copydoc Calibration::revert()
	 */
	virtual void revert();

private:
	rw::common::Ptr<T> findParameter(int parameterIndex, int& localParameterIndex) const;

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
void CompositeCalibration<T>::addCalibration(rw::common::Ptr<T>& calibration) {
	RW_ASSERT(!calibration->isApplied());
	_calibrations.push_back(calibration);
}

template<class T>
int CompositeCalibration<T>::getCalibrationCount() const {
	return _calibrations.size();
}

template<class T>
bool CompositeCalibration<T>::isEnabled() const {
	for (typename std::vector<rw::common::Ptr<T> >::const_iterator it = _calibrations.begin(); it != _calibrations.end(); ++it) {
		const rw::common::Ptr<T> calibration = (*it);
		if (calibration->isEnabled())
			return true;
	}

	return false;
}

template<class T>
void CompositeCalibration<T>::setEnabled(bool isEnabled) {
	for (typename std::vector<rw::common::Ptr<T> >::iterator it = _calibrations.begin(); it != _calibrations.end(); ++it) {
		rw::common::Ptr<T> calibration = (*it);
		calibration->setEnabled(isEnabled);
	}
}

template<class T>
double CompositeCalibration<T>::getParameterValue(int parameterIndex) const {
	int localParameterIndex;
	rw::common::Ptr<T> calibration = findParameter(parameterIndex, localParameterIndex);
	return calibration->getParameterValue(localParameterIndex);
}

template<class T>
void CompositeCalibration<T>::setParameterValue(int parameterIndex, double value) {
	int localParameterIndex;
	rw::common::Ptr<T> calibration = findParameter(parameterIndex, localParameterIndex);
	calibration->setParameterValue(localParameterIndex, value);
}

template<class T>
int CompositeCalibration<T>::getParameterCount() const {
	int parameterCount = 0;
	for (typename std::vector<rw::common::Ptr<T> >::const_iterator it = _calibrations.begin(); it != _calibrations.end(); ++it) {
		const rw::common::Ptr<T> calibration = (*it);
		parameterCount += calibration->getParameterCount();
	}
	return parameterCount;
}

template<class T>
int CompositeCalibration<T>::getEnabledParameterCount() const {
	if (!isEnabled())
		return 0;

	int enabledParameterCount = 0;
	for (typename std::vector<rw::common::Ptr<T> >::const_iterator it = _calibrations.begin(); it != _calibrations.end(); ++it) {
		const rw::common::Ptr<T> calibration = (*it);
		enabledParameterCount += calibration->getEnabledParameterCount();
	}
	return enabledParameterCount;
}

template<class T>
bool CompositeCalibration<T>::isParameterEnabled(int parameterIndex) const {
	int localParameterIndex;
	rw::common::Ptr<T> calibration = findParameter(parameterIndex, localParameterIndex);
	return calibration->isParameterEnabled(localParameterIndex);
}

template<class T>
void CompositeCalibration<T>::setParameterEnabled(int parameterIndex, bool isEnabled) {
	int localParameterIndex;
	rw::common::Ptr<T> calibration = findParameter(parameterIndex, localParameterIndex);
	calibration->setParameterEnabled(localParameterIndex, isEnabled);
}

template<class T>
bool CompositeCalibration<T>::isApplied() const {
	for (typename std::vector<rw::common::Ptr<T> >::const_iterator it = _calibrations.begin(); it != _calibrations.end(); ++it) {
		const rw::common::Ptr<T> calibration = (*it);
		if (calibration->isApplied())
			return true;
	}

	return false;
}

template<class T>
void CompositeCalibration<T>::apply() {
	RW_ASSERT(isEnabled());
	RW_ASSERT(!isApplied());

	for (typename std::vector<rw::common::Ptr<T> >::iterator it = _calibrations.begin(); it != _calibrations.end(); ++it) {
		rw::common::Ptr<T> calibration = (*it);
		if (calibration->isEnabled())
			calibration->apply();
	}

	RW_ASSERT(isApplied());
}

template<class T>
void CompositeCalibration<T>::revert() {
	RW_ASSERT(isEnabled());
	RW_ASSERT(isApplied());

	for (typename std::vector<rw::common::Ptr<T> >::iterator it = _calibrations.begin(); it != _calibrations.end(); ++it) {
		rw::common::Ptr<T> calibration = (*it);
		if (calibration->isApplied())
			calibration->revert();
	}

	RW_ASSERT(!isApplied());
}

template<class T>
rw::common::Ptr<T> CompositeCalibration<T>::findParameter(int parameterIndex, int& localParameterIndex) const {
	RW_ASSERT(parameterIndex < getParameterCount());

	rw::common::Ptr<T> calibration;
	int minParameterIndex = 0, maxParameterIndex = 0;
	for (typename std::vector<rw::common::Ptr<T> >::const_iterator it = _calibrations.begin(); it != _calibrations.end(); ++it) {
		calibration = (*it);
		const int parameterCount = calibration->getParameterCount();
		maxParameterIndex += parameterCount;
		if (parameterIndex >= minParameterIndex && parameterIndex < maxParameterIndex)
			break;
		minParameterIndex = maxParameterIndex;
	}

	RW_ASSERT(minParameterIndex != maxParameterIndex);

	localParameterIndex = parameterIndex - minParameterIndex;

	RW_ASSERT(localParameterIndex >= 0);

	return calibration;
}

/*@}*/
}
}

#endif /* RWLIBS_CALIBRATION_COMPOSITECALIBRATION_HPP_ */
