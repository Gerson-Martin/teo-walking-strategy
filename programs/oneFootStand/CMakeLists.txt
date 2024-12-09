if(NOT TARGET ROBOTICSLAB::KdlVectorConverterLib AND (NOT DEFINED ENABLE_oneFootStand OR ENABLE_oneFootStand))
    message(WARNING "KdlVectorConverterLib not found, disabling oneFootStand")
endif()

cmake_dependent_option(ENABLE_oneFootStand "Enable/disable oneFootStand program" ON
                       "TARGET ROBOTICSLAB::KdlVectorConverterLib" OFF)

if(ENABLE_oneFootStand)

    add_executable(oneFootStand main.cpp
                                OneFootStand.hpp
                                OneFootStand.cpp
                                walkingRobot.hpp
                                walkingRobot.cpp
                                BipedalController.hpp
                                BipedalController.cpp
                                LIPM2d.cpp
                                LIPM2d.h
                                global.h
                                utils.h)

    target_link_libraries(oneFootStand YARP::YARP_os
                                       YARP::YARP_init
                                       YARP::YARP_dev
                                       ${orocos_kdl_LIBRARIES}
                                       ROBOTICSLAB::KdlVectorConverterLib
                                       ROBOTICSLAB::KinematicsDynamicsInterfaces
                                       ${SUBDIR_LINK_NAMES})

    target_include_directories(oneFootStand PRIVATE ${orocos_kdl_INCLUDE_DIRS})

    install(TARGETS oneFootStand)

else()

    set(ENABLE_oneFootStand OFF CACHE BOOL "Enable/disable oneFootStand program" FORCE)

endif()
