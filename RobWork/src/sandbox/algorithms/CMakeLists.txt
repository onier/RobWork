SET(FILES_CPP
	ConstraintModel.cpp
	ConstraintGenerator.cpp
	PointConstraint.cpp
	LineConstraint.cpp
	StablePoseConstraint.cpp
	BoxConstraint.cpp
	PlaneConstraint.cpp
	FixtureConstraint.cpp
)

SET(FILES_HPP
    ConstraintModel.hpp
    ConstraintGenerator.hpp
    PointConstraint.hpp
    LineConstraint.hpp
    StablePoseConstraint.hpp
	BoxConstraint.hpp
	PlaneConstraint.hpp
	FixtureConstraint.hpp
)

SOURCE_GROUP(constraints FILES ${FILES_CPP} ${FILES_HPP})

FILE(GLOB SRC_FILES_CPP ${FILES_CPP})
FILE(GLOB SRC_FILES_HPP ${FILES_HPP})

SET(RW_LIB_FILES_CPP ${RW_LIB_FILES_CPP} ${SRC_FILES_CPP} PARENT_SCOPE)
SET(RW_LIB_FILES_HPP ${RW_LIB_FILES_HPP} ${SRC_FILES_HPP} PARENT_SCOPE)

#INSTALL(FILES ${CONSTRAINTS_FILES_HPP} DESTINATION "include/rw/constraints")
