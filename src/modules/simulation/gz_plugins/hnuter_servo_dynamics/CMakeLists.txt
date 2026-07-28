############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
#
############################################################################

project(HnuterServoDynamics)

add_library(${PROJECT_NAME} SHARED
    HnuterServoDynamics.cpp
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC ${GZ_PLUGIN_TARGET}
    PUBLIC ${GZ_SIM_TARGET}
    PUBLIC ${GZ_TRANSPORT_TARGET}
)

target_include_directories(${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
    PUBLIC ${CMAKE_CURRENT_BINARY_DIR}
)

if (NOT CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib/px4_gz_plugins)
endif()
