include(GNUInstallDirs)

add_library(libevents_parser
	parser.cpp
)

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
	target_compile_options(libevents_parser
		PRIVATE
		-Wall
		-Wextra
		-Werror
		-Wconversion
		-Wpedantic
	)
endif()
set_target_properties(libevents_parser PROPERTIES PUBLIC_HEADER parser.h)
install(TARGETS libevents_parser
		EXPORT libevents
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
		PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/libevents/parse)

add_library(nlohmann_json
		INTERFACE
		nlohmann/json_fwd.hpp
)
set_target_properties(nlohmann_json PROPERTIES PUBLIC_HEADER
		nlohmann/json_fwd.hpp)
install(TARGETS nlohmann_json
		EXPORT libevents
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
		PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/libevents/parse/nlohmann)

add_library(libevents_health_and_arming_checks
	health_and_arming_checks.cpp
)
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
	target_compile_options(libevents_health_and_arming_checks
		PRIVATE
		-Wall
		-Wextra
		-Werror
		-Wconversion
		-Wpedantic
	)
endif()
set_target_properties(libevents_health_and_arming_checks PROPERTIES PUBLIC_HEADER health_and_arming_checks.h)
install(TARGETS libevents_health_and_arming_checks
		EXPORT libevents
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
		PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/libevents/parse)
