include(GNUInstallDirs)

add_library(libevents_common INTERFACE
	event_type.h
)

set_target_properties(libevents_common PROPERTIES PUBLIC_HEADER event_type.h)
install(TARGETS libevents_common
		EXPORT libevents_common
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
		PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/libevents/common)
