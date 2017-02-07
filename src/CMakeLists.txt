add_definitions(
#    -ggdb3 
    -std=gnu99
    )

include_directories(
    ${GLIB2_INCLUDE_DIRS}
    ${LCMTYPES_INCLUDE_DIRS}
    )

add_executable(hr-robot-status main.c)

#target_link_libraries (hr-robot-status ${LCMTYPES_LIBS})

pods_use_pkg_config_packages(hr-robot-status
    glib-2.0
    gthread-2.0
    lcm 
    bot2-core 
    lcmtypes_hr-lcmtypes
    lcmtypes_bot2-core
    bot2-param-client)
    

pods_install_executables(hr-robot-status)
