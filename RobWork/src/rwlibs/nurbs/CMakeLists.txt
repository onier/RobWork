SET(SUBSYS_NAME rw_nurbs)
set(SUBSYS_DESC "NURBS interfaces for RobWork")
SET(SUBSYS_DEPS rw)

SET(build ON)
RW_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} OFF)
RW_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} EXT_DEPS openNURBS glu opengl)
RW_ADD_DOC(${SUBSYS_NAME})

IF( build )
    SET(SRC_FILES
      BezierCurveND.cpp
      BezierCurveRationalND.cpp
      BezierSurfaceND.cpp
      BezierSurfaceRationalND.cpp
      #BRep.cpp
      GeometricBezierCurve.cpp
      GeometricBezierSurface.cpp
      GeometricCurve.cpp
      GeometricNURBSCurve.cpp
      GeometricNURBSSurface.cpp
      GeometricSurface.cpp
      #NURBSCurve.cpp
      NUBSCurveND.cpp
      NURBSCurveND.cpp
      NUBSSurfaceND.cpp
      NURBSSurfaceND.cpp
      #ParametricCurve.cpp
      #ParametricPlaneFace.cpp
      #ParametricRoundedEdge.cpp
      #ParametricSurface.cpp
      ParametricUtility.cpp
      RenderBezierCurve.cpp
      RenderBezierSurface.cpp
      #RenderBRep.cpp
      RenderGeometricCurve.cpp
      RenderGeometricSurface.cpp
      RenderNURBSCurve.cpp
      RenderNURBSSurface.cpp
      #RenderParametricCurve.cpp
      #RenderParametricSurface.cpp
    )
    
    SET(SRC_FILES_HPP
      BezierCurveND.hpp
      BezierCurveRationalND.hpp
      BezierSurfaceND.hpp
      BezierSurfaceRationalND.hpp
      #BRep.hpp
      GeometricBezierCurve.hpp
      GeometricBezierSurface.hpp
      GeometricCurve.hpp
      GeometricNURBSCurve.hpp
      GeometricNURBSSurface.hpp
      GeometricSurface.hpp
      #NURBSCurve.hpp
      NUBSCurveND.hpp
      NURBSCurveND.hpp
      NUBSSurfaceND.hpp
      NURBSSurfaceND.hpp
      #ParametricCurve.hpp
      #ParametricPlaneFace.hpp
      #ParametricRoundedEdge.hpp
      #ParametricSurface.hpp
      ParametricUtility.hpp
      RenderBezierCurve.hpp
      RenderBezierSurface.hpp
      #RenderBRep.hpp
      RenderGeometricCurve.hpp
      RenderGeometricSurface.hpp
      RenderNURBSCurve.hpp
      RenderNURBSSurface.hpp
      #RenderParametricCurve.hpp
      #RenderParametricSurface.hpp
    )
    	
    RW_ADD_LIBRARY(${SUBSYS_NAME} nurbs ${SRC_FILES} ${SRC_FILES_HPP})
    RW_ADD_INCLUDES(nurbs "rwlibs/nurbs" ${SRC_FILES_HPP}) 
    TARGET_LINK_LIBRARIES(${SUBSYS_NAME} ${SUBSYS_DEPS})
    ADD_DEPENDENCIES(${SUBSYS_NAME} ${SUBSYS_DEPS})
ENDIF()