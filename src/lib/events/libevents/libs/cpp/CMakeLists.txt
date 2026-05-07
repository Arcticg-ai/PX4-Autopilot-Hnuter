cmake_minimum_required(VERSION 3.19)

if (POLICY CMP0048)
	cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)
project(libevents C CXX)

if (NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

include(GNUInstallDirs)

set(LIBEVENTS_CPP_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(LIBEVENTS_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../..")

# Determine if the library is built as a subproject (using add_subdirectory) or if it is the main project.
set(MAIN_PROJECT OFF)
if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
	set(MAIN_PROJECT ON)
endif()

option(ENABLE_TESTING "Enable building tests" ON)

if(MAIN_PROJECT AND ENABLE_TESTING)
	# compiler specific flags
	if (("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang") OR ("${CMAKE_CXX_COMPILER_ID}" MATCHES "AppleClang"))
		add_compile_options(
			-fcolor-diagnostics # force color for clang (needed for clang + ccache)
		)
	elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
		add_compile_options(-fdiagnostics-color=always)
	endif()

	add_subdirectory(tests)
endif()

add_subdirectory(common)
add_subdirectory(parse)
add_subdirectory(protocol)

add_library(libevents INTERFACE)
target_link_libraries(libevents INTERFACE libevents_parser libevents_health_and_arming_checks libevents_receive)
target_include_directories(libevents INTERFACE
		$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
install(TARGETS libevents
		EXPORT libevents
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
		PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/libevents)
install(EXPORT libevents
		FILE libevents.cmake
		DESTINATION lib/cmake/libevents
)
include(CMakePackageConfigHelpers)
# generate the config file that includes the exports
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in
		"${CMAKE_CURRENT_BINARY_DIR}/libeventsConfig.cmake"
		INSTALL_DESTINATION "lib/cmake/libevents"
		NO_SET_AND_CHECK_MACRO
		NO_CHECK_REQUIRED_COMPONENTS_MACRO
)
install(FILES
		${CMAKE_CURRENT_BINARY_DIR}/libeventsConfig.cmake
		DESTINATION lib/cmake/libevents
)
export(EXPORT libevents
		FILE "${CMAKE_CURRENT_BINARY_DIR}/libevents.cmake"
)

find_program(CLANG_FORMAT_EXECUTABLE
	NAMES clang-format clang-format-5.0
		clang-format-4.0 clang-format-3.9
		clang-format-3.8 clang-format-3.7
		clang-format-3.6 clang-format-3.5
		clang-format-3.4 clang-format-3.3
	DOC "clang-format executable"
)

file(GLOB LIBEVENTS_CPP_FORMAT_SRCS
	parse/*.cpp parse/*.h
	protocol/*.cpp protocol/*.h
	tests/*.cpp tests/*.h)
if (CLANG_FORMAT_EXECUTABLE)
	message(STATUS "Enabling format target")
	add_custom_target(
        format ${CMAKE_CURRENT_SOURCE_DIR}/scripts/run_clang_format.sh
		${CLANG_FORMAT_EXECUTABLE} ${LIBEVENTS_CPP_FORMAT_SRCS}
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		)
else()
    message(STATUS "Not enabling format target (clang-format not found)")
endif()

