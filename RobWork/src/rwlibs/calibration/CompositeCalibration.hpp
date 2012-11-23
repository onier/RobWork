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

	virtual void doCorrectState(rw::kinematics::State& state);

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
	RW_ASSERT(!isApplied());

	for (typename std::vector<rw::common::Ptr<T> >::iterator it = _calibrations.begin(); it != _calibrations.end(); ++it) {
		rw::common::Ptr<T> calibration = (*it);
		if (!calibration->isApplied())
			calibration->apply();
	}
}

template<class T>
void CompositeCalibration<T>::doRevert() {
	RW_ASSERT(isApplied());

	for (typename std::vector<rw::common::Ptr<T> >::iterator it = _calibrations.begin(); it != _calibrations.end(); ++it) {
		rw::common::Ptr<T> calibration = (*it);
		if (calibration->isApplied())
			calibration->revert();
	}
}

template<class T>
void CompositeCalibration<T>::doCorrectState(rw::kinematics::State& state) {
	RW_ASSERT(isApplied());

	for (typename std::vector<rw::common::Ptr<T> >::iterator it = _calibrations.begin(); it != _calibrations.end(); ++it) {
		rw::common::Ptr<T> calibration = (*it);
		if (calibration->isApplied())
			calibration->correctState(state);
	}
}

/*@}*/
}
}

#endif /* RWLIBS_CALIBRATION_COMPOSITECALIBRATION_HPP_ */
