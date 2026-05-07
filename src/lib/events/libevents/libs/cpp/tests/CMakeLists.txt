
# Avoid warning about DOWNLOAD_EXTRACT_TIMESTAMP in CMake 3.24:
if (POLICY CMP0135)
	cmake_policy(SET CMP0135 NEW)
endif()

include(FetchContent)
FetchContent_Declare(
	googletest
	URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

set(generated_events_header ${CMAKE_CURRENT_BINARY_DIR}/events_generated.h)
add_custom_command(OUTPUT ${generated_events_header}
	COMMAND ${PYTHON_EXECUTABLE} ${LIBEVENTS_ROOT_DIR}/scripts/generate.py
		--language cpp
		--output ${CMAKE_CURRENT_BINARY_DIR}
		${LIBEVENTS_ROOT_DIR}/libs/test.json
	DEPENDS
		${LIBEVENTS_ROOT_DIR}/libs/test.json
		${LIBEVENTS_ROOT_DIR}/scripts/generate.py
		${LIBEVENTS_ROOT_DIR}/libs/cpp/templates/events_generated.h.jinja
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	COMMENT "Generating test event header file"
)
add_custom_target(libevents_test_header DEPENDS ${generated_events_header})

add_executable(parser_test
	parser.cpp
)
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
	target_compile_options(parser_test
		PRIVATE
		-Wall
		-Wextra
		-Werror
		-Wconversion
		-Wpedantic
	)
endif()
target_link_libraries(parser_test PRIVATE
	libevents_parser
	GTest::gtest_main
)
target_include_directories(parser_test PRIVATE ${LIBEVENTS_CPP_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})
add_dependencies(parser_test libevents_test_header)

include(GoogleTest)
gtest_discover_tests(parser_test)
