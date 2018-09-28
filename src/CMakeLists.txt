add_definitions(
#    -ggdb3
    -std=gnu99
    )

include_directories(
    ${GLIB2_INCLUDE_DIRS}
    ${LCMTYPES_INCLUDE_DIRS}
    )

add_executable(robot-status main.c)

pods_use_pkg_config_packages(robot-status
    glib-2.0
    gthread-2.0
    lcm
    bot2-core
    lcmtypes_robot-status
    lcmtypes_bot2-core
    bot2-param-client)


pods_install_executables(robot-status)
