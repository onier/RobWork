SET(FILES_CPP
  AmbiguityResolver.cpp
  ClosedFormIK.cpp
  IterativeIK.cpp
  IterativeMultiIK.cpp
  CCDSolver.cpp
  ParallelIKSolver.cpp
  PieperSolver.cpp
  IKMetaSolver.cpp
  JacobianIKSolver.cpp
  JacobianIKSolverM.cpp
  ClosedFormIKSolverUR.cpp
  ClosedFormIKSolverKukaIIWA.cpp
)

SET(FILES_HPP
  AmbiguityResolver.hpp
  ClosedFormIK.hpp
  IterativeIK.hpp
  IterativeMultiIK.hpp
  CCDSolver.hpp
  ParallelIKSolver.hpp
  PieperSolver.hpp
  IKMetaSolver.hpp
  InvKinSolver.hpp
  JacobianIKSolver.hpp
  JacobianIKSolverM.hpp
  ClosedFormIKSolverUR.hpp
  ClosedFormIKSolverKukaIIWA.hpp
)

SOURCE_GROUP(invkin FILES ${FILES_CPP} ${FILES_HPP})

FILE(GLOB SRC_FILES_CPP ${FILES_CPP})
FILE(GLOB SRC_FILES_HPP ${FILES_HPP})

SET(RW_LIB_FILES_CPP ${RW_LIB_FILES_CPP} ${SRC_FILES_CPP} PARENT_SCOPE)
SET(RW_LIB_FILES_HPP ${RW_LIB_FILES_HPP} ${SRC_FILES_HPP} PARENT_SCOPE)

#INSTALL(FILES ${INVKIN_FILES_HPP} DESTINATION "include/rw/invkin")
