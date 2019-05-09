SET(SUBSYS_NAME rws_workcelleditor)
set(SUBSYS_DESC "A workcell editor plugin")
SET(SUBSYS_DEPS rws)

# if we want to use ui files add them here
SET(UIS_FILES WorkcellEditorWindow.ui)
IF(RWS_USE_QT5)
  QT5_WRAP_UI(UIS_OUT_H ${UIS_FILES})
ELSE()
  QT4_WRAP_UI(UIS_OUT_H ${UIS_FILES})
ENDIF()
#Need to add the current binary dir to the include directory because the generated source files are placed here
include_directories(${CMAKE_CURRENT_BINARY_DIR})

SET(SrcFiles WorkcellEditorWindow.cpp
        WorkcellHighlighter.cpp
        CodeEditor.cpp
        TreeModelCompleter.cpp
        InputFormDialog.cpp
        SetColorButton.cpp
        FileDialogButton.cpp
        GeometryComboBox.cpp
        )
SET(SRC_FILES_HPP WorkcellEditorWindow.hpp
        WorkcellHighlighter.hpp
        CodeEditor.hpp
        TreeModelCompleter.hpp
        InputFormDialog.hpp
        SetColorButton.hpp
        FileDialogButton.hpp
        GeometryComboBox.hpp
        )

set_source_files_properties(${SrcFiles} PROPERTIES OBJECT_DEPENDS "${UIS_OUT_H}")

IF(RWS_USE_QT5)
    QT5_WRAP_CPP(MocSrcFiles ${SRC_FILES_HPP})
ELSE()
# Header files to moc and compile:
    QT4_WRAP_CPP(MocSrcFiles ${SRC_FILES_HPP} OPTIONS -DBOOST_TT_HAS_PLUS_HPP_INCLUDED -DBOOST_TT_HAS_PLUS_ASSIGN_HPP_INCLUDED -DBOOST_TT_HAS_MINUS_HPP_INCLUDED -DBOOST_TT_HAS_MINUS_ASSIGN_HPP_INCLUDED)
ENDIF()

RWS_ADD_COMPONENT(${SUBSYS_NAME} components ${SrcFiles} ${MocSrcFiles} ${UIS_OUT_H})
RW_ADD_INCLUDES(components "rwslibs/workcelleditor" ${SRC_FILES_HPP})
TARGET_LINK_LIBRARIES(${SUBSYS_NAME} ${SUBSYS_DEPS})

IF(CMAKE_VERSION VERSION_GREATER 3.3)
    SET_TARGET_PROPERTIES(${SUBSYS_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
ENDIF()

IF(WIN32 AND MSVC AND PROJECT_LIB_TYPE AND CMAKE_VERSION VERSION_LESS 3.4)
    MESSAGE(FATAL_ERROR "Building shared libraries on Windows requires CMake 3.4 or newer.")
ENDIF()
