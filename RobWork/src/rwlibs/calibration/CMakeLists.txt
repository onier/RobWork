
SET( SUBSYS_NAME rw_calibration )
SET( SUBSYS_DESC "Calibration of devices" )
SET( SUBSYS_DEPS rw )

SET( build ${RW_BUILD_CALIBRATION} )
RW_SUBSYS_OPTION( build ${SUBSYS_NAME} ${SUBSYS_DESC} ${RW_BUILD_CALIBRATION} )
RW_SUBSYS_DEPEND( build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} )
RW_ADD_DOC( ${SUBSYS_NAME} )

IF( build )
	SET(SRC_FILES
		nlls/NLLSIterationLog.cpp
		nlls/NLLSSolver.cpp
		nlls/NLLSSolverLog.cpp
		nlls/NLLSSystem.cpp
		xml/XmlCalibrationLoader.cpp
		xml/XmlCalibrationSaver.cpp
		xml/XmlMeasurementFile.cpp
		Calibration.cpp
		CompositeCalibration.cpp
		DHParameterCalibration.cpp
	#	EncoderParameterCalibration.cpp
		FixedFrameCalibration.cpp
		SerialDeviceCalibration.cpp
		SerialDeviceCalibrator.cpp
		SerialDevicePoseMeasurement.cpp
	)
    
	SET(SRC_FILES_HPP
		nlls/NLLSIterationLog.hpp
		nlls/NLLSSolver.hpp
		nlls/NLLSSolverLog.hpp
		nlls/NLLSSystem.hpp
		xml/XmlCalibrationLoader.hpp
		xml/XmlCalibrationSaver.hpp
		xml/XmlMeasurementFile.hpp
		Calibration.hpp
		CompositeCalibration.hpp
		DHParameterCalibration.hpp
		EigenTransformAddons.hpp
	#	EncoderParameterCalibration.hpp
		FixedFrameCalibration.hpp
		Pose6D.hpp
		RPY.hpp
		SerialDeviceCalibration.hpp
		SerialDeviceCalibrator.hpp
		SerialDevicePoseMeasurement.hpp
	)
    
	RW_ADD_LIBRARY( ${SUBSYS_NAME} calibration ${SRC_FILES} ${SRC_FILES_HPP} )
	RW_ADD_INCLUDES( calibration "rwlibs/calibration" ${SRC_FILES_HPP} ) 
	TARGET_LINK_LIBRARIES( ${SUBSYS_NAME} ${SUBSYS_DEPS} )
	ADD_DEPENDENCIES( ${SUBSYS_NAME} ${SUBSYS_DEPS} )
ENDIF()
