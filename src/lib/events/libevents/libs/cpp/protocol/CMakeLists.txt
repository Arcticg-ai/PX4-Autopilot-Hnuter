include(GNUInstallDirs)

add_library(libevents_receive INTERFACE
	receive.h
)

set_target_properties(libevents_receive PROPERTIES PUBLIC_HEADER receive.h)
install(TARGETS libevents_receive
		EXPORT libevents
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
		PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/libevents/protocol)
