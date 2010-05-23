/*
 * SimulatedScanner25D.hpp
 *
 *  Created on: 23/05/2010
 *      Author: jimali
 */

#ifndef RWLIBS_SIMULATION_SIMULATEDSCANNER25D_HPP_
#define RWLIBS_SIMULATION_SIMULATEDSCANNER25D_HPP_

#include <rw/sensor/Scanner25D.hpp>
#include "FrameGrabber25D.hpp"
#include "SimulatedSensor.hpp"

namespace rwlibs { namespace simulation {
    /** @addtogroup simulation */
    /* @{ */

    /**
       @brief
    */
    class SimulatedScanner25D : public rw::sensor::Scanner25D, public virtual SimulatedSensor
    {
    public:

    	SimulatedScanner25D(const std::string& name,
    			FrameGrabber25DPtr framegrabber);

    	SimulatedScanner25D(const std::string& name,
    			const std::string& desc,
    			FrameGrabber25DPtr framegrabber);

    	virtual ~SimulatedScanner25D();

    	void setFrameRate(double rate);

    	///////////// below is inheritet functions form Scanner25D and Sensor

    	//! @copydoc Scanner25D::open
        void open();

        //! @copydoc Scanner25D::isOpen
        bool isOpen();

        //! @copydoc Scanner25D::close
        void close();

        //! @copydoc Scanner25D::acquire
        void acquire();

        //! @copydoc Scanner25D::isScanReady
        bool isScanReady();

        //! @copydoc Scanner25D::getRange
        std::pair<double,double> getRange();

        //! @copydoc Scanner25D::getFrameRate
        double getFrameRate();

        //! @copydoc Scanner25D::getImage
    	const rw::sensor::Image25D& getImage();

    	//! @copydoc SimulatedSensor::update
        void update(double dt, rw::kinematics::State& state);

    	//! @copydoc SimulatedSensor::reset
    	void reset(const rw::kinematics::State& state);

    	//! @copydoc SimulatedSensor::getSensor
        rw::sensor::Sensor* getSensor();

    private:
        FrameGrabber25DPtr _framegrabber;
        double _frameRate, _dtsum;
        bool _isAcquired,_isOpenned;
    };
}
}

#endif /* SIMULATEDSCANNER25D_HPP_ */
