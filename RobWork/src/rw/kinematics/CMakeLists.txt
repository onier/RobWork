SET(FILES_CPP
  FixedFrame.cpp
  FKRange.cpp
  FKTable.cpp
  Frame.cpp
  FrameType.cpp
  FrameMap.cpp
  Kinematics.cpp
  MovableFrame.cpp
  State.cpp
  QState.cpp
  TreeState.cpp
  StateStructure.cpp
  StateData.cpp
  StateSetup.cpp
  FramePairMap.cpp
  StatelessData.cpp
  Stateless.cpp
)

SET(FILES_HPP
  FixedFrame.hpp
  FKRange.hpp
  FKTable.hpp
  Frame.hpp
  FrameType.hpp
  FrameMap.hpp
  Kinematics.hpp
  MovableFrame.hpp
  State.hpp
  QState.hpp
  TreeState.hpp
  StateStructure.hpp
  StateData.hpp
  StateSetup.hpp
  FramePairMap.hpp
  StatelessData.hpp
  Stateless.hpp
)

SOURCE_GROUP(kinematics FILES ${FILES_CPP} ${FILES_HPP})

FILE(GLOB SRC_FILES_CPP ${FILES_CPP})
FILE(GLOB SRC_FILES_HPP ${FILES_HPP})

SET(RW_LIB_FILES_CPP ${RW_LIB_FILES_CPP} ${SRC_FILES_CPP} PARENT_SCOPE)
SET(RW_LIB_FILES_HPP ${RW_LIB_FILES_HPP} ${SRC_FILES_HPP} PARENT_SCOPE)

#INSTALL(FILES ${KINEMATICS_FILES_HPP} DESTINATION "include/rw/kinematics/")
