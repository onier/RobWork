cmake_minimum_required(VERSION 3.5)
if (POLICY CMP0074)
	cmake_policy(SET CMP0074 NEW)
endif()
list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# options
option(CODE "Build primary libraries and executables." ON)
option(PYTHON_BINDINGS "Build python bindings for libraries" ON)

if(NOT DEFINED DOC)
	set(DOC OFF CACHE STRING
			"The documentation type to generate." FORCE)
endif()

# disable base languages
unset(PROJECT_LANGUAGES)

if(${CODE})
	set(PROJECT_LANGUAGES ${PROJECT_LANGUAGES} C CXX)
	set(CMAKE_CXX_STANDARD 11)
	set(CMAKE_CXX_STANDARD_REQUIRED ON)
	set(CMAKE_CXX_EXTENSIONS OFF)
	if(DEFINED WIN32)
		ADD_DEFINITIONS(-D_WIN32_WINNT=0x0A00)
		ADD_DEFINITIONS(-DBOOST_ALL_NO_LIB)
	endif()
endif()

project(ur_rtde VERSION 1.0.0 LANGUAGES ${PROJECT_LANGUAGES})
set(PROJECT_VERSION_SUFFIX "") # alpha/beta/rc, e.g. "-rc0"
set(PROJECT_VERSION "${PROJECT_VERSION}${PROJECT_VERSION_SUFFIX}")
set(PROJECT_AUTHOR "Anders Prier Lindvig")
set(PROJECT_COPYRIGHT "2019")
set(PROJECT_MAIL "anpl@mmmi.sdu.dk")

if(${CODE})
	set(Boost_USE_MULTITHREAD ON)
	find_package(Boost REQUIRED COMPONENTS system thread)
	find_package(Threads)

	if(${PYTHON_BINDINGS})
		find_package(pybind11 REQUIRED)
	endif()
endif()

set(LIB_SOURCE_FILES
        src/rtde.cpp
        src/dashboard_client.cpp
        src/script_client.cpp
        src/rtde_control_interface.cpp
		src/rtde_receive_interface.cpp)

set(LIB_HEADER_FILES
        include/rtde.h
        include/rtde_utility.h
        include/dashboard_client.h
        include/robot_state.h
        include/script_client.h
        include/rtde_control_interface.h
		include/rtde_receive_interface.h)

if(${PYTHON_BINDINGS})
	find_package(pybind11 REQUIRED)
endif()

if(NOT DOC STREQUAL "OFF")
	add_subdirectory(doc)
endif()

if(${CODE})
	include(GenerateExportHeader)

	# Write the rtde_control script to a header file
	file(READ scripts/rtde_control.script RTDE_CONTROL_SCRIPT)
	set(header "const std::string NEW_LINE= \"\\n\"; const std::string QUOTATION = \"\\\"\"; std::string UR_SCRIPT = \"")
	string(REGEX REPLACE "\"" "\"+QUOTATION+\"" FILEVAR ${RTDE_CONTROL_SCRIPT})
	string(REGEX REPLACE "\\\n" "\" + NEW_LINE  + \n\"" FILEVAR ${FILEVAR})
	set(end "\";")
	file(WRITE include/rtde_control_script.h "${header}${FILEVAR}${end}")

	add_library(rtde SHARED ${LIB_SOURCE_FILES} ${LIB_HEADER_FILES})
	target_include_directories(rtde
			PUBLIC ${Boost_INCLUDE_DIRS}
			$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
			$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
			$<INSTALL_INTERFACE:include>)
	target_link_libraries(rtde PUBLIC
			${Boost_SYSTEM_LIBRARY}
			${Boost_THREAD_LIBRARY}
			${CMAKE_THREAD_LIBS_INIT})

	if(${PYTHON_BINDINGS})
		pybind11_add_module(rtde_control SHARED ${LIB_SOURCE_FILES} ${LIB_HEADER_FILES}	src/rtde_python_bindings.cpp)
		target_include_directories(rtde_control
				PUBLIC ${Boost_INCLUDE_DIRS}
				$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
				$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
				$<INSTALL_INTERFACE:include>
				PRIVATE ${PYBIND11_INCLUDE_DIR}  # from project CMakeLists.txt
				PRIVATE ${pybind11_INCLUDE_DIR}  # from pybind11Config
				PRIVATE ${PYTHON_INCLUDE_DIRS})
		target_link_libraries(rtde_control PRIVATE
				${PYTHON_LIBRARIES}
				${Boost_SYSTEM_LIBRARY}
				${Boost_THREAD_LIBRARY}
				${CMAKE_THREAD_LIBS_INIT})
                target_compile_definitions(rtde_control PRIVATE rtde_EXPORTS)

		pybind11_add_module(rtde_receive SHARED ${LIB_SOURCE_FILES} ${LIB_HEADER_FILES} src/rtde_python_bindings.cpp)
		target_include_directories(rtde_receive
			PUBLIC ${Boost_INCLUDE_DIRS}
			$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
			$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
			$<INSTALL_INTERFACE:include>
			PRIVATE ${PYBIND11_INCLUDE_DIR}  # from project CMakeLists.txt
			PRIVATE ${pybind11_INCLUDE_DIR}  # from pybind11Config
			PRIVATE ${PYTHON_INCLUDE_DIRS})
		target_link_libraries(rtde_receive PRIVATE
				${PYTHON_LIBRARIES}
				${Boost_SYSTEM_LIBRARY}
				${Boost_THREAD_LIBRARY})
                target_compile_definitions(rtde_receive PRIVATE rtde_EXPORTS)
	endif()

	set(RTDE_RECEIVE_CLIENT_SOURCE_FILES src/rtde_receive_client.cpp)
	add_executable(rtde_receive_client ${RTDE_RECEIVE_CLIENT_SOURCE_FILES})
	target_include_directories(rtde_receive_client
			PUBLIC ${Boost_INCLUDE_DIRS} $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
	target_link_libraries(rtde_receive_client PRIVATE
			rtde
			${Boost_SYSTEM_LIBRARY}
			${Boost_THREAD_LIBRARY})

	set(RTDE_SEND_CLIENT_SOURCE_FILES src/rtde_send_client.cpp)
	add_executable(rtde_send_client ${RTDE_SEND_CLIENT_SOURCE_FILES})
	target_include_directories(rtde_send_client
			PUBLIC ${Boost_INCLUDE_DIRS} $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
	target_link_libraries(rtde_send_client PRIVATE
			rtde
			${Boost_SYSTEM_LIBRARY}
			${Boost_THREAD_LIBRARY})

	generate_export_header(rtde)

	add_library(ur_rtde::rtde ALIAS rtde)
	add_executable(ur_rtde::rtde_receive_client ALIAS rtde_receive_client)
	add_executable(ur_rtde::rtde_send_client ALIAS rtde_send_client)

	if (CMAKE_COMPILER_IS_GNUCXX)
		# Give error in Linux if library is not fully linked (like Windows)
		set_target_properties(rtde PROPERTIES LINK_FLAGS -Wl,--no-undefined)
	endif()

	install(TARGETS rtde
			EXPORT RTDETargets
			LIBRARY DESTINATION lib
			ARCHIVE DESTINATION lib
			RUNTIME DESTINATION bin
			INCLUDES DESTINATION include
			)
	if(${PYTHON_BINDINGS})
		install(TARGETS rtde_control rtde_receive
				DESTINATION ${PYTHON_SITE_PACKAGES})
	endif()

	install(
			FILES
			${LIB_HEADER_FILES}
			"${CMAKE_CURRENT_BINARY_DIR}/rtde_export.h"
			DESTINATION include
			COMPONENT ur_rtde
	)

	include(CMakePackageConfigHelpers)
	write_basic_package_version_file(
		"${CMAKE_CURRENT_BINARY_DIR}/ur_rtde/ur_rtdeConfigVersion.cmake"
		VERSION ${PROJECT_VERSION}
		COMPATIBILITY AnyNewerVersion
	)
	export(EXPORT RTDETargets
		FILE "${CMAKE_CURRENT_BINARY_DIR}/ur_rtde/ur_rtdeTargets.cmake"
		NAMESPACE ur_rtde::
	)
	configure_file(cmake/ur_rtdeConfig.cmake
		"${CMAKE_CURRENT_BINARY_DIR}/ur_rtde/ur_rtdeConfig.cmake"
		COPYONLY
	)
	install(EXPORT RTDETargets
		FILE ur_rtdeTargets.cmake
		NAMESPACE ur_rtde::
		DESTINATION lib/cmake/ur_rtde
	)
	install(
		FILES cmake/ur_rtdeConfig.cmake
		"${CMAKE_CURRENT_BINARY_DIR}/ur_rtde/ur_rtdeConfigVersion.cmake"
		DESTINATION lib/cmake/ur_rtde
		COMPONENT ur_rtde
	)
endif()
